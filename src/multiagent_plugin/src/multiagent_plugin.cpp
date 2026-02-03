/*
 Copyright (c) 2025 Composiv.ai

 This program and the accompanying materials are made available under the
 terms of the Eclipse Public License 2.0 which is available at
 http://www.eclipse.org/legal/epl-2.0.

 SPDX-License-Identifier: EPL-2.0

 Contributors:
   Composiv.ai - initial API and implementation
*/

#include "multiagent_plugin.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QTimer>
#include <QFont>
#include <QFrame>

namespace multiagent_plugin
{
    MultiagentPanel::MultiagentPanel(QWidget *parent)
        : rviz_common::Panel(parent),
          simulation_running_(false),
          simulation_paused_(false),
          spawn_all_mode_(false),
          algorithm_active_(false)
    {
        // Create ROS node first to read parameters
        node_ = std::make_shared<rclcpp::Node>("multiagent_plugin_node");
        
        // Get num_agents parameter (default: 3)
        node_->declare_parameter("num_agents", 3);
        num_agents_ = node_->get_parameter("num_agents").as_int();

        // Setup stylesheet for modern look
        setupStylesheet();

        // Main layout
        QVBoxLayout *main_layout = new QVBoxLayout;
        main_layout->setSpacing(10);

        // === STATUS SECTION ===
        QGroupBox *status_group = new QGroupBox("Status", this);
        QHBoxLayout *status_layout = new QHBoxLayout;
        
        status_led_ = new QFrame(this);
        status_led_->setFixedSize(20, 20);
        status_led_->setFrameShape(QFrame::Box);
        status_led_->setStyleSheet("background-color: #888888; border-radius: 10px; border: 1px solid #555;");
        
        status_label_ = new QLabel("Waiting for algorithm...", this);
        status_label_->setStyleSheet("font-weight: bold;");
        
        status_layout->addWidget(status_led_);
        status_layout->addWidget(status_label_);
        status_layout->addStretch();
        status_group->setLayout(status_layout);
        main_layout->addWidget(status_group);

        // === RACE CONTROL SECTION ===
        QGroupBox *control_group = new QGroupBox("Race Control", this);
        QVBoxLayout *control_layout = new QVBoxLayout;
        
        QHBoxLayout *buttons_row1 = new QHBoxLayout;
        start_button_ = new QPushButton("▶ Start", this);
        start_button_->setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 8px; } QPushButton:hover { background-color: #45a049; }");
        
        pause_button_ = new QPushButton("⏸ Pause", this);
        pause_button_->setStyleSheet("QPushButton { background-color: #FF9800; color: white; font-weight: bold; padding: 8px; } QPushButton:hover { background-color: #e68a00; }");
        
        buttons_row1->addWidget(start_button_);
        buttons_row1->addWidget(pause_button_);
        
        QHBoxLayout *buttons_row2 = new QHBoxLayout;
        reset_button_ = new QPushButton("↺ Reset", this);
        reset_button_->setStyleSheet("QPushButton { background-color: #f44336; color: white; font-weight: bold; padding: 8px; } QPushButton:hover { background-color: #da190b; }");
        
        spawn_all_button_ = new QPushButton("⊕ Spawn All", this);
        spawn_all_button_->setStyleSheet("QPushButton { background-color: #2196F3; color: white; font-weight: bold; padding: 8px; } QPushButton:hover { background-color: #1976D2; }");
        spawn_all_button_->setCheckable(true);  // Toggle button
        
        buttons_row2->addWidget(reset_button_);
        buttons_row2->addWidget(spawn_all_button_);
        
        QHBoxLayout *buttons_row3 = new QHBoxLayout;
        clear_obstacles_button_ = new QPushButton("🗑 Clear Obstacles", this);
        clear_obstacles_button_->setStyleSheet("QPushButton { background-color: #9E9E9E; color: white; font-weight: bold; padding: 8px; } QPushButton:hover { background-color: #757575; }");
        buttons_row3->addWidget(clear_obstacles_button_);
        
        control_layout->addLayout(buttons_row1);
        control_layout->addLayout(buttons_row2);
        control_layout->addLayout(buttons_row3);
        control_group->setLayout(control_layout);
        main_layout->addWidget(control_group);

        // === RACECAR SELECTION SECTION ===
        QGroupBox *agent_group = new QGroupBox("Racecar Selection", this);
        QVBoxLayout *agent_layout = new QVBoxLayout;
        
        QLabel *agent_label = new QLabel("Select racecar for pose estimation:", this);
        agent_dropdown_ = new QComboBox(this);
        
        // Dynamically populate dropdown based on num_agents
        for (int i = 1; i <= num_agents_; i++) {
            agent_dropdown_->addItem(QString("Racecar %1").arg(i));
        }
        agent_dropdown_->addItem("Set Lap Point");
        
        agent_layout->addWidget(agent_label);
        agent_layout->addWidget(agent_dropdown_);
        agent_group->setLayout(agent_layout);
        main_layout->addWidget(agent_group);

        // === LAP TIMES SECTION ===
        QGroupBox *lap_group = new QGroupBox("Lap Times", this);
        QVBoxLayout *lap_layout = new QVBoxLayout;
        
        lap_times_label_ = new QLabel("", this);
        QFont mono_font("Monospace");
        mono_font.setStyleHint(QFont::TypeWriter);
        mono_font.setPointSize(9);
        lap_times_label_->setFont(mono_font);
        lap_times_label_->setStyleSheet("background-color: #1e1e1e; color: #00ff00; padding: 8px; border-radius: 4px;");
        lap_times_label_->setMinimumHeight(80);
        
        lap_layout->addWidget(lap_times_label_);
        lap_group->setLayout(lap_layout);
        main_layout->addWidget(lap_group);

        main_layout->addStretch();
        setLayout(main_layout);

        // Setup ROS publishers and subscribers
        start_publisher_ = node_->create_publisher<std_msgs::msg::Bool>("sim_start", 10);
        pause_publisher_ = node_->create_publisher<std_msgs::msg::Bool>("sim_pause", 10);
        reset_publisher_ = node_->create_publisher<std_msgs::msg::Bool>("sim_reset", 10);
        agent_publisher_ = node_->create_publisher<std_msgs::msg::Int32>("racecar_to_estimate_pose", 10);
        spawn_all_publisher_ = node_->create_publisher<std_msgs::msg::Bool>("spawn_all_mode", 10);
        clear_obstacles_publisher_ = node_->create_publisher<std_msgs::msg::Bool>("clear_obstacles", 10);
        
        lap_times_subscriber_ = node_->create_subscription<std_msgs::msg::String>(
            "lap_times", 10, std::bind(&MultiagentPanel::updateLapTimesLabel, this, std::placeholders::_1));
        
        race_stats_subscriber_ = node_->create_subscription<std_msgs::msg::String>(
            "race_stats_json", 10, std::bind(&MultiagentPanel::updateRaceStats, this, std::placeholders::_1));

        // ROS spin timer
        QTimer *ros_spin_timer_ = new QTimer(this);
        connect(ros_spin_timer_, SIGNAL(timeout()), this, SLOT(spinSome()));
        ros_spin_timer_->start(100);
        
        // Connect signals - use activated instead of currentIndexChanged to fire on every selection
        connect(agent_dropdown_, SIGNAL(activated(int)), this, SLOT(onAgentSelected(int)));
        connect(start_button_, SIGNAL(clicked()), this, SLOT(onStartButtonClicked()));
        connect(pause_button_, SIGNAL(clicked()), this, SLOT(onPauseButtonClicked()));
        connect(reset_button_, SIGNAL(clicked()), this, SLOT(onResetButtonClicked()));
        connect(spawn_all_button_, SIGNAL(toggled(bool)), this, SLOT(onSpawnAllToggled(bool)));
        connect(clear_obstacles_button_, SIGNAL(clicked()), this, SLOT(onClearObstaclesClicked()));
        
        // Publish initial agent selection (index 0 = Racecar 1)
        QTimer::singleShot(500, this, [this]() {
            std_msgs::msg::Int32 msg;
            msg.data = 0;
            agent_publisher_->publish(msg);
        });
    }

    void MultiagentPanel::setupStylesheet()
    {
        setStyleSheet(
            "QGroupBox { font-weight: bold; border: 1px solid #555; border-radius: 5px; margin-top: 10px; padding-top: 10px; }"
            "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }"
        );
    }

    void MultiagentPanel::updateStatusIndicator(const QString &state, const QString &color)
    {
        status_label_->setText(state);
        status_led_->setStyleSheet(QString("background-color: %1; border-radius: 10px; border: 1px solid #555;").arg(color));
    }

    void MultiagentPanel::spinSome()
    {
        rclcpp::spin_some(node_);
    }

    void MultiagentPanel::updateLapTimesLabel(const std_msgs::msg::String::SharedPtr msg)
    {
        lap_times_label_->setText(QString::fromStdString(msg->data));
        
        // If we're receiving lap times, algorithm is active
        if (!algorithm_active_) {
            algorithm_active_ = true;
        }
    }

    void MultiagentPanel::updateRaceStats(const std_msgs::msg::String::SharedPtr msg)
    {
        // Simple string parsing to check states
        std::string data = msg->data;
        bool running = data.find("\"simulation_running\": true") != std::string::npos ||
                       data.find("\"simulation_running\":true") != std::string::npos;
        bool paused = data.find("\"simulation_paused\": true") != std::string::npos ||
                      data.find("\"simulation_paused\":true") != std::string::npos;
        
        // Check if any agent has non-zero speed (algorithm is driving)
        bool has_speed = data.find("\"speed\": 0.0") == std::string::npos &&
                         data.find("\"speed\":0.0") == std::string::npos &&
                         data.find("\"speed\": 0,") == std::string::npos;
        
        if (!running) {
            updateStatusIndicator("Stopped", "#888888");
        } else if (paused) {
            updateStatusIndicator("Paused", "#FF9800");
        } else if (!has_speed && !algorithm_active_) {
            updateStatusIndicator("Waiting for algorithm...", "#888888");
        } else {
            updateStatusIndicator("Racing", "#4CAF50");
            algorithm_active_ = true;
        }
        
        simulation_running_ = running;
        simulation_paused_ = paused;
    }

    MultiagentPanel::~MultiagentPanel() {}

    void MultiagentPanel::onAgentSelected(int index)
    {
        // Disable spawn_all mode when manually selecting an agent
        if (spawn_all_mode_) {
            spawn_all_button_->setChecked(false);
        }
        
        std_msgs::msg::Int32 msg;
        msg.data = index;
        agent_publisher_->publish(msg);
    }

    void MultiagentPanel::onStartButtonClicked()
    {
        std_msgs::msg::Bool msg;
        msg.data = true;
        start_publisher_->publish(msg);
        simulation_running_ = true;
        updateStatusIndicator("Racing", "#4CAF50");
    }

    void MultiagentPanel::onResetButtonClicked()
    {
        std_msgs::msg::Bool msg;
        msg.data = true;
        reset_publisher_->publish(msg);
        algorithm_active_ = false;
    }

    void MultiagentPanel::onPauseButtonClicked()
    {
        std_msgs::msg::Bool msg;
        msg.data = true;
        pause_publisher_->publish(msg);
        
        if (simulation_paused_) {
            updateStatusIndicator("Racing", "#4CAF50");
        } else {
            updateStatusIndicator("Paused", "#FF9800");
        }
    }

    void MultiagentPanel::onSpawnAllToggled(bool checked)
    {
        spawn_all_mode_ = checked;
        
        // Update button appearance
        if (checked) {
            spawn_all_button_->setStyleSheet("QPushButton { background-color: #1565C0; color: white; font-weight: bold; padding: 8px; border: 2px solid #0D47A1; }");
            spawn_all_button_->setText("⊕ Spawn All (ACTIVE)");
        } else {
            spawn_all_button_->setStyleSheet("QPushButton { background-color: #2196F3; color: white; font-weight: bold; padding: 8px; } QPushButton:hover { background-color: #1976D2; }");
            spawn_all_button_->setText("⊕ Spawn All");
        }
        
        // Publish spawn_all mode state
        std_msgs::msg::Bool msg;
        msg.data = checked;
        spawn_all_publisher_->publish(msg);
    }

    void MultiagentPanel::onClearObstaclesClicked()
    {
        std_msgs::msg::Bool msg;
        msg.data = true;
        clear_obstacles_publisher_->publish(msg);
    }

} // namespace multiagent_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(multiagent_plugin::MultiagentPanel, rviz_common::Panel)
