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
#include <QTimer>
#include <QGroupBox>

namespace multiagent_plugin
{
    MultiagentPanel::MultiagentPanel(QWidget *parent)
        : rviz_common::Panel(parent),
          simulation_running_(false),
          simulation_paused_(false)
    {
        // Main layout
        QVBoxLayout *main_layout = new QVBoxLayout;
        main_layout->setSpacing(10);
        main_layout->setContentsMargins(10, 10, 10, 10);

        // === Status Section ===
        status_group_ = new QGroupBox("Simulation Status", this);
        QHBoxLayout *status_layout = new QHBoxLayout;

        status_indicator_ = new QLabel(this);
        status_indicator_->setFixedSize(16, 16);
        status_indicator_->setStyleSheet("background-color: #ff4444; border-radius: 8px;");

        status_label_ = new QLabel("Stopped", this);
        status_label_->setStyleSheet("font-weight: bold;");

        status_layout->addWidget(status_indicator_);
        status_layout->addWidget(status_label_);
        status_layout->addStretch();
        status_group_->setLayout(status_layout);

        // === Race Control Section ===
        control_group_ = new QGroupBox("Race Control", this);
        QVBoxLayout *control_layout = new QVBoxLayout;

        start_button_ = new QPushButton("Start Race", this);
        start_button_->setMinimumHeight(40);
        start_button_->setStyleSheet(
            "QPushButton {"
            "  background-color: #4CAF50;"
            "  color: white;"
            "  border: none;"
            "  border-radius: 5px;"
            "  font-weight: bold;"
            "}"
            "QPushButton:hover { background-color: #45a049; }"
            "QPushButton:pressed { background-color: #3d8b40; }"
            "QPushButton:disabled { background-color: #888888; }"
        );

        pause_button_ = new QPushButton("Pause Race", this);
        pause_button_->setMinimumHeight(35);
        pause_button_->setStyleSheet(
            "QPushButton {"
            "  background-color: #2196F3;"
            "  color: white;"
            "  border: none;"
            "  border-radius: 5px;"
            "}"
            "QPushButton:hover { background-color: #1976D2; }"
        );

        reset_button_ = new QPushButton("Reset Simulation", this);
        reset_button_->setMinimumHeight(35);
        reset_button_->setStyleSheet(
            "QPushButton {"
            "  background-color: #ff9800;"
            "  color: white;"
            "  border: none;"
            "  border-radius: 5px;"
            "}"
            "QPushButton:hover { background-color: #f57c00; }"
        );

        control_layout->addWidget(start_button_);
        control_layout->addWidget(pause_button_);
        control_layout->addWidget(reset_button_);
        control_group_->setLayout(control_layout);

        // === Agent Selection Section ===
        agent_group_ = new QGroupBox("Agent Selection", this);
        QVBoxLayout *agent_layout = new QVBoxLayout;

        QLabel *agent_label = new QLabel("Select agent for pose estimation:", this);
        agent_dropdown_ = new QComboBox(this);
        agent_dropdown_->setMinimumHeight(30);

        // Add agents dynamically (supports up to 20)
        for (int i = 1; i <= 10; i++) {
            agent_dropdown_->addItem(QString("Agent%1").arg(i));
        }
        agent_dropdown_->addItem("Set Lap Point");

        agent_layout->addWidget(agent_label);
        agent_layout->addWidget(agent_dropdown_);
        agent_group_->setLayout(agent_layout);

        // === Lap Times Section ===
        lap_group_ = new QGroupBox("Lap Times", this);
        QVBoxLayout *lap_layout = new QVBoxLayout;

        lap_times_label_ = new QLabel("Waiting for race to start...", this);
        lap_times_label_->setStyleSheet(
            "font-family: monospace;"
            "background-color: #1e1e1e;"
            "color: #00ff00;"
            "padding: 10px;"
            "border-radius: 5px;"
        );
        lap_times_label_->setMinimumHeight(80);

        lap_layout->addWidget(lap_times_label_);
        lap_group_->setLayout(lap_layout);

        // Assemble main layout
        main_layout->addWidget(status_group_);
        main_layout->addWidget(control_group_);
        main_layout->addWidget(agent_group_);
        main_layout->addWidget(lap_group_);
        main_layout->addStretch();

        setLayout(main_layout);

        // === ROS Setup ===
        node_ = std::make_shared<rclcpp::Node>("multiagent_plugin_node");

        lap_times_subscriber_ = node_->create_subscription<std_msgs::msg::String>(
            "lap_times", 10,
            std::bind(&MultiagentPanel::updateLapTimesLabel, this, std::placeholders::_1));

        start_publisher_ = node_->create_publisher<std_msgs::msg::Bool>("sim_start", 10);
        pause_publisher_ = node_->create_publisher<std_msgs::msg::Bool>("sim_pause", 10);
        reset_publisher_ = node_->create_publisher<std_msgs::msg::Bool>("sim_reset", 10);
        agent_publisher_ = node_->create_publisher<std_msgs::msg::Int32>("racecar_to_estimate_pose", 10);

        // Timer for ROS spin
        QTimer *ros_spin_timer_ = new QTimer(this);
        connect(ros_spin_timer_, SIGNAL(timeout()), this, SLOT(spinSome()));
        ros_spin_timer_->start(100);

        // Connect signals
        connect(agent_dropdown_, SIGNAL(currentIndexChanged(int)), this, SLOT(onAgentSelected(int)));
        connect(start_button_, SIGNAL(clicked()), this, SLOT(onStartButtonClicked()));
        connect(pause_button_, SIGNAL(clicked()), this, SLOT(onPauseButtonClicked()));
        connect(reset_button_, SIGNAL(clicked()), this, SLOT(onResetButtonClicked()));

        applyStyles();
    }

    void MultiagentPanel::applyStyles()
    {
        // Apply consistent styling to group boxes
        QString groupBoxStyle =
            "QGroupBox {"
            "  font-weight: bold;"
            "  border: 1px solid #555;"
            "  border-radius: 5px;"
            "  margin-top: 10px;"
            "  padding-top: 10px;"
            "}"
            "QGroupBox::title {"
            "  subcontrol-origin: margin;"
            "  left: 10px;"
            "  padding: 0 5px;"
            "}";

        status_group_->setStyleSheet(groupBoxStyle);
        control_group_->setStyleSheet(groupBoxStyle);
        agent_group_->setStyleSheet(groupBoxStyle);
        lap_group_->setStyleSheet(groupBoxStyle);
    }

    void MultiagentPanel::updateStatusIndicator()
    {
        if (!simulation_running_) {
            status_indicator_->setStyleSheet("background-color: #ff4444; border-radius: 8px;");
            status_label_->setText("Stopped");
            start_button_->setText("Start Race");
            start_button_->setEnabled(true);
            pause_button_->setText("Pause Race");
        } else if (simulation_paused_) {
            status_indicator_->setStyleSheet("background-color: #ffaa00; border-radius: 8px;");
            status_label_->setText("Paused");
            pause_button_->setText("Resume Race");
        } else {
            status_indicator_->setStyleSheet("background-color: #44ff44; border-radius: 8px;");
            status_label_->setText("Running");
            start_button_->setText("Race Started");
            start_button_->setEnabled(false);
            pause_button_->setText("Pause Race");
        }
    }

    void MultiagentPanel::spinSome()
    {
        rclcpp::spin_some(node_);
    }

    void MultiagentPanel::updateLapTimesLabel(const std_msgs::msg::String::SharedPtr msg)
    {
        lap_times_label_->setText(QString::fromStdString(msg->data));
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

        simulation_running_ = true;
        simulation_paused_ = false;
        updateStatusIndicator();
    }

    void MultiagentPanel::onResetButtonClicked()
    {
        std_msgs::msg::Bool msg;
        msg.data = true;
        reset_publisher_->publish(msg);

        simulation_running_ = false;
        simulation_paused_ = false;
        updateStatusIndicator();
        lap_times_label_->setText("Simulation reset. Waiting for race to start...");
    }

    void MultiagentPanel::onPauseButtonClicked()
    {
        std_msgs::msg::Bool msg;
        msg.data = true;
        pause_publisher_->publish(msg);

        if (simulation_running_) {
            simulation_paused_ = !simulation_paused_;
            updateStatusIndicator();
        }
    }

} // namespace multiagent_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(multiagent_plugin::MultiagentPanel, rviz_common::Panel)
