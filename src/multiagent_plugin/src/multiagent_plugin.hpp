/*
 Copyright (c) 2025 Composiv.ai

 This program and the accompanying materials are made available under the
 terms of the Eclipse Public License 2.0 which is available at
 http://www.eclipse.org/legal/epl-2.0.

 SPDX-License-Identifier: EPL-2.0

 Contributors:
   Composiv.ai - initial API and implementation
*/

#ifndef MULTIAGENT_PLUGIN_HPP_
#define MULTIAGENT_PLUGIN_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <rviz_common/panel.hpp>
#include <QPushButton>
#include <QLabel>
#include <QGroupBox>
#include <QFrame>
#include <QGridLayout>
#include <QButtonGroup>
#include <vector>

namespace multiagent_plugin
{
    class MultiagentPanel : public rviz_common::Panel
    {
        Q_OBJECT
    public:
        MultiagentPanel(QWidget *parent = nullptr);
        virtual ~MultiagentPanel() override;

    protected:
        // UI Elements - Status Section
        QFrame *status_led_;
        QLabel *status_label_;
        
        // UI Elements - Race Control
        QPushButton *start_button_;
        QPushButton *pause_button_;
        QPushButton *reset_button_;
        QPushButton *spawn_all_button_;
        QPushButton *spawn_obstacle_button_;
        QPushButton *set_lap_point_button_;
        QPushButton *clear_obstacles_button_;
        
        // UI Elements - Racecar Selection (numbered buttons)
        QButtonGroup *racecar_button_group_;
        std::vector<QPushButton*> racecar_buttons_;
        
        // UI Elements - Lap Time Cards
        QGridLayout *lap_cards_layout_;
        std::vector<QFrame*> lap_cards_;
        
        // ROS Node
        rclcpp::Node::SharedPtr node_;
        int num_agents_;
        
        // Publishers
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_publisher_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reset_publisher_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pause_publisher_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr agent_publisher_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr spawn_all_publisher_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr spawn_obstacle_publisher_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr set_lap_point_publisher_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr clear_obstacles_publisher_;
        
        // Subscribers
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr race_stats_subscriber_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_feedback_subscriber_;
        
        // State tracking
        bool simulation_running_;
        bool simulation_paused_;
        bool spawn_all_mode_;
        bool spawn_obstacle_mode_;
        bool set_lap_point_mode_;
        bool drive_ever_received_;
        int selected_racecar_;
        
        // Helper methods
        void updateStatusIndicator(const QString &state, const QString &color);
        void updateLapTimeCards(const std::string &data);
        void setupStylesheet();

    protected Q_SLOTS:
        void onStartButtonClicked();
        void onResetButtonClicked();
        void onPauseButtonClicked();
        void onSpawnAllToggled(bool checked);
        void onSpawnObstacleToggled(bool checked);
        void onSetLapPointToggled(bool checked);
        void onClearObstaclesClicked();
        void onRacecarSelected(int index);
        void spinSome();
        
    private:
        void updateRaceStats(const std_msgs::msg::String::SharedPtr msg);
        void onModeFeedback(const std_msgs::msg::String::SharedPtr msg);
    };

} // namespace multiagent_plugin

#endif // MULTIAGENT_PLUGIN_HPP_
