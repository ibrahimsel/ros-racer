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
#include <QGridLayout>
#include <QGroupBox>
#include <QTimer>
#include <QFont>
#include <QFrame>
#include <QButtonGroup>
#include <cstdlib>
#include <string>
#include <sstream>
#include <vector>

namespace multiagent_plugin
{
    MultiagentPanel::MultiagentPanel(QWidget *parent)
        : rviz_common::Panel(parent),
          simulation_running_(false),
          simulation_paused_(false),
          spawn_all_mode_(false),
          spawn_obstacle_mode_(false),
          set_lap_point_mode_(false),
          drive_ever_received_(false),
          selected_racecar_(0)
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
        
        // Row 1: Start, Pause
        QHBoxLayout *buttons_row1 = new QHBoxLayout;
        start_button_ = new QPushButton("▶ Start", this);
        start_button_->setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 8px; } QPushButton:hover { background-color: #45a049; }");
        
        pause_button_ = new QPushButton("⏸ Pause", this);
        pause_button_->setStyleSheet("QPushButton { background-color: #FF9800; color: white; font-weight: bold; padding: 8px; } QPushButton:hover { background-color: #e68a00; }");
        
        buttons_row1->addWidget(start_button_);
        buttons_row1->addWidget(pause_button_);
        
        // Row 2: Reset, Clear Obstacles
        QHBoxLayout *buttons_row2 = new QHBoxLayout;
        reset_button_ = new QPushButton("↺ Reset", this);
        reset_button_->setStyleSheet("QPushButton { background-color: #f44336; color: white; font-weight: bold; padding: 8px; } QPushButton:hover { background-color: #da190b; }");
        
        clear_obstacles_button_ = new QPushButton("🗑 Clear Obstacles", this);
        clear_obstacles_button_->setStyleSheet("QPushButton { background-color: #9E9E9E; color: white; font-weight: bold; padding: 8px; } QPushButton:hover { background-color: #757575; }");
        
        buttons_row2->addWidget(reset_button_);
        buttons_row2->addWidget(clear_obstacles_button_);
        
        // Row 3: Spawn All, Spawn Obstacle, Set Lap Point (toggle buttons)
        QHBoxLayout *buttons_row3 = new QHBoxLayout;
        
        spawn_all_button_ = new QPushButton("⊕ Spawn All", this);
        spawn_all_button_->setCheckable(true);
        spawn_all_button_->setStyleSheet("QPushButton { background-color: #2196F3; color: white; font-weight: bold; padding: 6px; } QPushButton:hover { background-color: #1976D2; } QPushButton:checked { background-color: #1565C0; border: 2px solid #0D47A1; }");
        
        spawn_obstacle_button_ = new QPushButton("⬤ Obstacle", this);
        spawn_obstacle_button_->setCheckable(true);
        spawn_obstacle_button_->setStyleSheet("QPushButton { background-color: #9C27B0; color: white; font-weight: bold; padding: 6px; } QPushButton:hover { background-color: #7B1FA2; } QPushButton:checked { background-color: #6A1B9A; border: 2px solid #4A148C; }");
        
        set_lap_point_button_ = new QPushButton("🏁 Lap Point", this);
        set_lap_point_button_->setCheckable(true);
        set_lap_point_button_->setStyleSheet("QPushButton { background-color: #607D8B; color: white; font-weight: bold; padding: 6px; } QPushButton:hover { background-color: #546E7A; } QPushButton:checked { background-color: #455A64; border: 2px solid #37474F; }");
        
        buttons_row3->addWidget(spawn_all_button_);
        buttons_row3->addWidget(spawn_obstacle_button_);
        buttons_row3->addWidget(set_lap_point_button_);
        
        control_layout->addLayout(buttons_row1);
        control_layout->addLayout(buttons_row2);
        control_layout->addLayout(buttons_row3);
        control_group->setLayout(control_layout);
        main_layout->addWidget(control_group);

        // === RACECAR SELECTION SECTION ===
        QGroupBox *agent_group = new QGroupBox("Racecar Selection", this);
        QVBoxLayout *agent_layout = new QVBoxLayout;
        
        QLabel *agent_label = new QLabel("Select racecar for pose estimation:", this);
        agent_layout->addWidget(agent_label);
        
        // Create numbered buttons for racecar selection
        QHBoxLayout *racecar_buttons_layout = new QHBoxLayout;
        racecar_button_group_ = new QButtonGroup(this);
        racecar_button_group_->setExclusive(true);
        
        for (int i = 0; i < num_agents_; i++) {
            QPushButton *btn = new QPushButton(QString::number(i + 1), this);
            btn->setCheckable(true);
            btn->setFixedSize(40, 40);
            btn->setStyleSheet("QPushButton { background-color: #424242; color: white; font-weight: bold; font-size: 14px; border-radius: 5px; } QPushButton:hover { background-color: #616161; } QPushButton:checked { background-color: #2196F3; border: 2px solid #1565C0; }");
            racecar_button_group_->addButton(btn, i);
            racecar_buttons_.push_back(btn);
            racecar_buttons_layout->addWidget(btn);
        }
        racecar_buttons_layout->addStretch();
        
        // Select first racecar by default
        if (!racecar_buttons_.empty()) {
            racecar_buttons_[0]->setChecked(true);
        }
        
        agent_layout->addLayout(racecar_buttons_layout);
        agent_group->setLayout(agent_layout);
        main_layout->addWidget(agent_group);

        // === LAP TIMES SECTION ===
        QGroupBox *lap_group = new QGroupBox("Lap Times", this);
        lap_cards_layout_ = new QGridLayout;
        lap_cards_layout_->setSpacing(8);
        
        // Create lap time cards for each racecar
        for (int i = 0; i < num_agents_; i++) {
            QFrame *card = new QFrame(this);
            card->setFrameShape(QFrame::Box);
            card->setStyleSheet("QFrame { background-color: #2d2d2d; border-radius: 5px; padding: 5px; }");
            
            QVBoxLayout *card_layout = new QVBoxLayout(card);
            card_layout->setSpacing(2);
            card_layout->setContentsMargins(8, 8, 8, 8);
            
            QLabel *name_label = new QLabel(QString("Racecar %1").arg(i + 1), card);
            name_label->setStyleSheet("font-weight: bold; color: #ffffff; font-size: 11px;");
            
            QLabel *current_label = new QLabel("Current: --", card);
            current_label->setStyleSheet("color: #00ff00; font-size: 10px;");
            current_label->setObjectName("current");
            
            QLabel *best_label = new QLabel("Best: --", card);
            best_label->setStyleSheet("color: #ffff00; font-size: 10px;");
            best_label->setObjectName("best");
            
            card_layout->addWidget(name_label);
            card_layout->addWidget(current_label);
            card_layout->addWidget(best_label);
            
            lap_cards_.push_back(card);
            
            // Grid layout: 2 columns (flow horizontally, wrap to next row)
            int row = i / 2;
            int col = i % 2;
            lap_cards_layout_->addWidget(card, row, col);
        }
        
        lap_group->setLayout(lap_cards_layout_);
        main_layout->addWidget(lap_group);

        main_layout->addStretch();
        setLayout(main_layout);

        // Setup ROS publishers
        start_publisher_ = node_->create_publisher<std_msgs::msg::Bool>("sim_start", 10);
        pause_publisher_ = node_->create_publisher<std_msgs::msg::Bool>("sim_pause", 10);
        reset_publisher_ = node_->create_publisher<std_msgs::msg::Bool>("sim_reset", 10);
        agent_publisher_ = node_->create_publisher<std_msgs::msg::Int32>("racecar_to_estimate_pose", 10);
        spawn_all_publisher_ = node_->create_publisher<std_msgs::msg::Bool>("spawn_all_mode", 10);
        spawn_obstacle_publisher_ = node_->create_publisher<std_msgs::msg::Bool>("spawn_obstacle_mode", 10);
        set_lap_point_publisher_ = node_->create_publisher<std_msgs::msg::Bool>("set_lap_point_mode", 10);
        clear_obstacles_publisher_ = node_->create_publisher<std_msgs::msg::Bool>("clear_obstacles", 10);
        
        // Setup ROS subscribers
        race_stats_subscriber_ = node_->create_subscription<std_msgs::msg::String>(
            "race_stats_json", 10, std::bind(&MultiagentPanel::updateRaceStats, this, std::placeholders::_1));
        
        mode_feedback_subscriber_ = node_->create_subscription<std_msgs::msg::String>(
            "mode_feedback", 10, std::bind(&MultiagentPanel::onModeFeedback, this, std::placeholders::_1));

        // ROS spin timer
        QTimer *ros_spin_timer_ = new QTimer(this);
        connect(ros_spin_timer_, SIGNAL(timeout()), this, SLOT(spinSome()));
        ros_spin_timer_->start(100);
        
        // Connect signals
        connect(start_button_, SIGNAL(clicked()), this, SLOT(onStartButtonClicked()));
        connect(pause_button_, SIGNAL(clicked()), this, SLOT(onPauseButtonClicked()));
        connect(reset_button_, SIGNAL(clicked()), this, SLOT(onResetButtonClicked()));
        connect(spawn_all_button_, SIGNAL(toggled(bool)), this, SLOT(onSpawnAllToggled(bool)));
        connect(spawn_obstacle_button_, SIGNAL(toggled(bool)), this, SLOT(onSpawnObstacleToggled(bool)));
        connect(set_lap_point_button_, SIGNAL(toggled(bool)), this, SLOT(onSetLapPointToggled(bool)));
        connect(clear_obstacles_button_, SIGNAL(clicked()), this, SLOT(onClearObstaclesClicked()));
        connect(racecar_button_group_, SIGNAL(idClicked(int)), this, SLOT(onRacecarSelected(int)));
        
        // Publish initial racecar selection
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

    void MultiagentPanel::onModeFeedback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string feedback = msg->data;
        
        if (feedback == "spawn_all_complete") {
            spawn_all_mode_ = false;
            spawn_all_button_->setChecked(false);
        } else if (feedback == "spawn_obstacle_complete") {
            spawn_obstacle_mode_ = false;
            spawn_obstacle_button_->setChecked(false);
        } else if (feedback == "lap_point_complete") {
            set_lap_point_mode_ = false;
            set_lap_point_button_->setChecked(false);
        }
    }

    void MultiagentPanel::updateRaceStats(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string data = msg->data;
        
        // Parse simulation state
        bool running = data.find("\"simulation_running\": true") != std::string::npos ||
                       data.find("\"simulation_running\":true") != std::string::npos;
        bool paused = data.find("\"simulation_paused\": true") != std::string::npos ||
                      data.find("\"simulation_paused\":true") != std::string::npos;
        
        // Parse speed values to detect if any car is moving
        bool any_moving = false;
        size_t pos = 0;
        while ((pos = data.find("\"speed\"", pos)) != std::string::npos) {
            pos = data.find(":", pos);
            if (pos == std::string::npos) break;
            pos++;
            while (pos < data.size() && (data[pos] == ' ' || data[pos] == '\t')) pos++;
            double speed = 0.0;
            try {
                speed = std::stod(data.substr(pos));
            } catch (...) {
                speed = 0.0;
            }
            if (speed > 0.1) {
                any_moving = true;
                drive_ever_received_ = true;  // Mark that we've seen movement
                break;
            }
        }
        
        // Update status indicator
        if (!running) {
            updateStatusIndicator("Stopped", "#888888");
        } else if (paused) {
            updateStatusIndicator("Paused", "#FF9800");
        } else if (any_moving) {
            updateStatusIndicator("Racing", "#4CAF50");
        } else if (!drive_ever_received_) {
            updateStatusIndicator("Waiting for algorithm...", "#888888");
        } else {
            updateStatusIndicator("Idle", "#FFC107");
        }
        
        simulation_running_ = running;
        simulation_paused_ = paused;
        
        // Update lap time cards
        updateLapTimeCards(data);
    }

    void MultiagentPanel::updateLapTimeCards(const std::string &data)
    {
        // Parse each agent's data from JSON
        size_t agents_pos = data.find("\"agents\"");
        if (agents_pos == std::string::npos) return;
        
        for (int i = 0; i < num_agents_ && i < (int)lap_cards_.size(); i++) {
            // Find this agent's data block
            std::string agent_marker = "\"id\": " + std::to_string(i + 1);
            size_t agent_pos = data.find(agent_marker);
            if (agent_pos == std::string::npos) continue;
            
            // Parse current_lap_time
            double current_time = 0.0;
            size_t current_pos = data.find("\"current_lap_time\"", agent_pos);
            if (current_pos != std::string::npos && current_pos < agent_pos + 500) {
                current_pos = data.find(":", current_pos);
                if (current_pos != std::string::npos) {
                    try {
                        current_time = std::stod(data.substr(current_pos + 1));
                    } catch (...) {}
                }
            }
            
            // Parse best_lap_time
            double best_time = 0.0;
            size_t best_pos = data.find("\"best_lap_time\"", agent_pos);
            if (best_pos != std::string::npos && best_pos < agent_pos + 500) {
                best_pos = data.find(":", best_pos);
                if (best_pos != std::string::npos) {
                    try {
                        best_time = std::stod(data.substr(best_pos + 1));
                    } catch (...) {}
                }
            }
            
            // Parse disqualified
            bool disqualified = false;
            size_t dq_pos = data.find("\"disqualified\"", agent_pos);
            if (dq_pos != std::string::npos && dq_pos < agent_pos + 500) {
                disqualified = data.find("true", dq_pos) < dq_pos + 30;
            }
            
            // Update card labels
            QFrame *card = lap_cards_[i];
            QLabel *current_label = card->findChild<QLabel*>("current");
            QLabel *best_label = card->findChild<QLabel*>("best");
            
            if (current_label) {
                current_label->setText(QString("Current: %1s").arg(current_time, 0, 'f', 2));
            }
            if (best_label) {
                if (best_time > 0) {
                    best_label->setText(QString("Best: %1s").arg(best_time, 0, 'f', 2));
                } else {
                    best_label->setText("Best: --");
                }
            }
            
            // Update card background based on disqualification
            if (disqualified) {
                card->setStyleSheet("QFrame { background-color: #8B0000; border-radius: 5px; padding: 5px; }");
            } else {
                card->setStyleSheet("QFrame { background-color: #2d2d2d; border-radius: 5px; padding: 5px; }");
            }
        }
    }

    MultiagentPanel::~MultiagentPanel() {}

    void MultiagentPanel::onRacecarSelected(int index)
    {
        selected_racecar_ = index;
        
        // Disable all toggle modes when selecting a racecar
        if (spawn_all_mode_) {
            spawn_all_mode_ = false;
            spawn_all_button_->setChecked(false);
        }
        if (spawn_obstacle_mode_) {
            spawn_obstacle_mode_ = false;
            spawn_obstacle_button_->setChecked(false);
        }
        if (set_lap_point_mode_) {
            set_lap_point_mode_ = false;
            set_lap_point_button_->setChecked(false);
        }
        
        // Publish selected racecar
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
    }

    void MultiagentPanel::onResetButtonClicked()
    {
        std_msgs::msg::Bool msg;
        msg.data = true;
        reset_publisher_->publish(msg);
        drive_ever_received_ = false;  // Reset the flag
        updateStatusIndicator("Waiting for algorithm...", "#888888");
    }

    void MultiagentPanel::onPauseButtonClicked()
    {
        std_msgs::msg::Bool msg;
        msg.data = true;
        pause_publisher_->publish(msg);
    }

    void MultiagentPanel::onSpawnAllToggled(bool checked)
    {
        spawn_all_mode_ = checked;
        
        // Disable other modes
        if (checked) {
            if (spawn_obstacle_mode_) {
                spawn_obstacle_mode_ = false;
                spawn_obstacle_button_->setChecked(false);
            }
            if (set_lap_point_mode_) {
                set_lap_point_mode_ = false;
                set_lap_point_button_->setChecked(false);
            }
        }
        
        std_msgs::msg::Bool msg;
        msg.data = checked;
        spawn_all_publisher_->publish(msg);
    }

    void MultiagentPanel::onSpawnObstacleToggled(bool checked)
    {
        spawn_obstacle_mode_ = checked;
        
        // Disable other modes
        if (checked) {
            if (spawn_all_mode_) {
                spawn_all_mode_ = false;
                spawn_all_button_->setChecked(false);
            }
            if (set_lap_point_mode_) {
                set_lap_point_mode_ = false;
                set_lap_point_button_->setChecked(false);
            }
        }
        
        std_msgs::msg::Bool msg;
        msg.data = checked;
        spawn_obstacle_publisher_->publish(msg);
    }

    void MultiagentPanel::onSetLapPointToggled(bool checked)
    {
        set_lap_point_mode_ = checked;
        
        // Disable other modes
        if (checked) {
            if (spawn_all_mode_) {
                spawn_all_mode_ = false;
                spawn_all_button_->setChecked(false);
            }
            if (spawn_obstacle_mode_) {
                spawn_obstacle_mode_ = false;
                spawn_obstacle_button_->setChecked(false);
            }
        }
        
        std_msgs::msg::Bool msg;
        msg.data = checked;
        set_lap_point_publisher_->publish(msg);
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
