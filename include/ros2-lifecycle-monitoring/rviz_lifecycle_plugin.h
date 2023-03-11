#pragma once

#include <QtWidgets>
#include <QHBoxLayout>

#include <thread>
#include <ctime>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

#include "cmr_clients_utils/basic_action_client.hpp"
#include "cmr_clients_utils/basic_service_client.hpp"

namespace rviz_lifecycle_plugin
{
    class RvizLifecyclePlugin : public rviz_common::Panel
    {
        Q_OBJECT
    public:
        explicit RvizLifecyclePlugin(QWidget *parent = 0);
        virtual ~RvizLifecyclePlugin();

    private:
        void get_lifecycle_node_names(std::vector<std::string>& lifecycle_node_names);
        void get_lifecycle_nodes_statuses(const std::vector<std::string>& nodes_names);
        void monitoring();

        void onInitialize() override;

        // ROS
        rclcpp::Node::SharedPtr utility_node_;
        std::shared_ptr<cmr_clients_utils::BasicServiceClient<lifecycle_msgs::srv::GetState>> lifecycle_get_state_client_;

        QListWidget *node_names_;

        std::shared_ptr<std::thread> monitoring_thread_;

        // RosbagPlay
        const QString rosbag_paused_ = "<table><tr><td width=100><b>Leg Filter:</b></td>"
                                           "<td><font color=green>active</color></td></tr></table>";
        const QString rosbag_play_available_ = "<table><tr><td width=100><b>Rosbag Play:</b></td>"
                                              "<td><font color=green>available</color></td></tr></table>";
        const QString rosbag_play_debug_ = "<table><tr><td width=100><b>Rosbag Play:</b></td>"
                                              "<td><font color=green>call sent</color></td></tr></table>";
        const QString rosbag_play_debug2_ = "<table><tr><td width=100><b>Rosbag Play:</b></td>"
                                              "<td><font color=green>debug2</color></td></tr></table>";
        const QString rosbag_play_not_running_ = "<table><tr><td width=100><b>Rosbag Play:</b></td>"
                                            "<td><font color=orange>not running</color></td></tr></table>";
        const QString rosbag_play_unknown_ = "<table><tr><td width=100><b>Rosbag Play:</b></td>"
                                                "<td><font color=orange>unknown</color></td></tr></table>";
    };
}

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_lifecycle_plugin::RvizLifecyclePlugin, rviz_common::Panel)