#pragma once

#include <QtWidgets>
#include <QHBoxLayout>

#include <thread>
#include <ctime>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "cmr_clients_utils/basic_action_client.hpp"
#include "cmr_clients_utils/basic_service_client.hpp"

namespace rviz_lifecycle_plugin
{
    typedef typename cmr_clients_utils::BasicServiceClient<lifecycle_msgs::srv::GetState> GetStateClient;
    typedef typename lifecycle_msgs::msg::State LifecycleState;

    class RvizLifecyclePlugin : public rviz_common::Panel
    {
        Q_OBJECT
    public:
        explicit RvizLifecyclePlugin(QWidget *parent = 0);
        virtual ~RvizLifecyclePlugin();

    private:
        
        /*
        *  Returns the fully qualified name of the lifecycle nodes
        */
        void get_lifecycle_node_names(std::vector<std::string>& lifecycle_node_names);
        
        /*
        *  Returns the status of the lifecycle nodes
        */
        LifecycleState get_lifecycle_node_state(const std::string& node_name);


        /*
        *  Updates the table widget at given row. If row does not exist, a new row is created
        */
        void update_table_widget(const size_t row, const std::string& node_name, const LifecycleState& state);

        /*
        * Creates a client for the serivce /node_name/get_state and stores it in the clients_ map
        */
        void add_client(const std::string& fully_qualified_name);
        void monitoring();

        void get_node_name_and_namespace(
            const std::string& fully_qualified_name,
            std::string& name,
            std::string& name_space);

        void onInitialize() override;

        // ROS
        rclcpp::Node::SharedPtr utility_node_;

        // QT
        QVBoxLayout *main_layout_;
        QTableWidget *node_names_;
        QScrollArea *scroll_area_;
        
        // store fully qualified node names
        std::vector<std::string> lifecycle_nodes_;
        std::unordered_map<std::string, std::shared_ptr<GetStateClient>> clients_;

        // store the state of the lifecycle nodes to be able to provide it immediately by request, future development of CLI tool
        std::unordered_map<std::string, LifecycleState> lifecycle_node_states_;

        std::shared_ptr<std::thread> monitoring_thread_;
        const std::chrono::milliseconds monitoring_interval_ = std::chrono::milliseconds(1000);
    };
}

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_lifecycle_plugin::RvizLifecyclePlugin, rviz_common::Panel)