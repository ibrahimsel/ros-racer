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
          simulation_paused_(false)
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
        
        status_label_ = new QLabel("Waiting...", this);
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
        
        buttons_row2->addWidget(reset_button_);
        buttons_row2->addWidget(spawn_all_button_);
        
        control_layout->addLayout(buttons_row1);
        control_layout->addLayout(buttons_row2);
        control_group->setLayout(control_layout);
        main_layout->addWidget(control_group);

        // === AGENT SELECTION SECTION ===
        QGroupBox *agent_group = new QGroupBox("Agent Selection", this);
        QVBoxLayout *agent_layout = new QVBoxLayout;
        
        QLabel *agent_label = new QLabel("Choose Pose Estimation:", this);
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
        pose_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
        
        lap_times_subscriber_ = node_->create_subscription<std_msgs::msg::String>(
            "lap_times", 10, std::bind(&MultiagentPanel::updateLapTimesLabel, this, std::placeholders::_1));
        
        race_stats_subscriber_ = node_->create_subscription<std_msgs::msg::String>(
            "race_stats_json", 10, std::bind(&MultiagentPanel::updateRaceStats, this, std::placeholders::_1));

        // ROS spin timer
        QTimer *ros_spin_timer_ = new QTimer(this);
        connect(ros_spin_timer_, SIGNAL(timeout()), this, SLOT(spinSome()));
        ros_spin_timer_->start(100);
        
        // Connect signals
        connect(agent_dropdown_, SIGNAL(currentIndexChanged(int)), this, SLOT(onAgentSelected(int)));
        connect(start_button_, SIGNAL(clicked()), this, SLOT(onStartButtonClicked()));
        connect(pause_button_, SIGNAL(clicked()), this, SLOT(onPauseButtonClicked()));
        connect(reset_button_, SIGNAL(clicked()), this, SLOT(onResetButtonClicked()));
        connect(spawn_all_button_, SIGNAL(clicked()), this, SLOT(onSpawnAllClicked()));
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
    }

    void MultiagentPanel::updateRaceStats(const std_msgs::msg::String::SharedPtr msg)
    {
        // Simple string parsing to avoid nlohmann/json dependency
        std::string data = msg->data;
        bool running = data.find("\"simulation_running\": true") != std::string::npos ||
                       data.find("\"simulation_running\":true") != std::string::npos;
        bool paused = data.find("\"simulation_paused\": true") != std::string::npos ||
                      data.find("\"simulation_paused\":true") != std::string::npos;
        
        if (!running) {
            updateStatusIndicator("Stopped", "#888888");
        } else if (paused) {
            updateStatusIndicator("Paused", "#FF9800");
        } else {
            updateStatusIndicator("Racing", "#4CAF50");
        }
        
        simulation_running_ = running;
        simulation_paused_ = paused;
    }

    MultiagentPanel::~MultiagentPanel() {}

    void MultiagentPanel::onAgentSelected(int index)
    {
        std_msgs::msg::Int32 msg;
        msg.data = index;
        agent_publisher_->publish(msg);
    }

    void MultiagentPanel::onStartButtonClicked()
    {
        std_msgs::msg::Bool msg;
        msg.data = true;
        start_publisher_->publish(msg);
        updateStatusIndicator("Racing", "#4CAF50");
    }

    void MultiagentPanel::onResetButtonClicked()
    {
        std_msgs::msg::Bool msg;
        msg.data = true;
        reset_publisher_->publish(msg);
    }

    void MultiagentPanel::onPauseButtonClicked()
    {
        std_msgs::msg::Bool msg;
        msg.data = true;
        pause_publisher_->publish(msg);
    }

    void MultiagentPanel::onSpawnAllClicked()
    {
        // Spawn all agents in a line formation at the start
        for (int i = 0; i < num_agents_; i++) {
            // Select the agent
            std_msgs::msg::Int32 agent_msg;
            agent_msg.data = i;
            agent_publisher_->publish(agent_msg);
            
            // Small delay to allow processing
            rclcpp::sleep_for(std::chrono::milliseconds(50));
            
            // Publish initial pose (agents in a vertical line, 1m apart)
            geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
            pose_msg.header.stamp = node_->now();
            pose_msg.header.frame_id = "map";
            pose_msg.pose.pose.position.x = static_cast<double>(i);
            pose_msg.pose.pose.position.y = 0.0;
            pose_msg.pose.pose.position.z = 0.0;
            pose_msg.pose.pose.orientation.x = 0.0;
            pose_msg.pose.pose.orientation.y = 0.0;
            pose_msg.pose.pose.orientation.z = 0.0;
            pose_msg.pose.pose.orientation.w = 1.0;
            pose_publisher_->publish(pose_msg);
            
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }
    }

} // namespace multiagent_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(multiagent_plugin::MultiagentPanel, rviz_common::Panel)
