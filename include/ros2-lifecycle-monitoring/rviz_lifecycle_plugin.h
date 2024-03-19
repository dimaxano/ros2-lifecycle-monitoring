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

namespace rviz_lifecycle_plugin
{
    typedef typename rclcpp::Client<lifecycle_msgs::srv::GetState> GetStateClient;
    typedef typename lifecycle_msgs::msg::State LifecycleState;

    class RvizLifecyclePlugin : public rviz_common::Panel
    {
        Q_OBJECT
    public:
        explicit RvizLifecyclePlugin(QWidget *parent = 0);
        virtual ~RvizLifecyclePlugin();

    private:
        
        /*
        *  Update list clients to fetch the state of the lifecycle nodes
        */
        void update_lifecycle_clients();
        
        /*
        *  Requests the status of the lifecycle node
        */
        void request_lifecycle_node_state(const std::string& node_name);

        /*
        *  Updates the table widget at given row. If row does not exist, a new row is created
        */
        void update_table_widget(const size_t row, const std::string& node_name, const LifecycleState& state);

        /*
        *  Set color of the label according to the state of the lifecycle node
        */
        void set_label_color(QLabel* label, const LifecycleState& state);

        void monitoring();
        void update_ui();

        void get_node_name_and_namespace(
            const std::string& fully_qualified_name,
            std::string& name,
            std::string& name_space);

        void onInitialize() override;

        // ROS
        rclcpp::Node::SharedPtr utility_node_;

        // QT
        QVBoxLayout *main_layout_;
        QTableWidget *nodes_states_table_;
        QScrollArea *scroll_area_;
        QThread* thread_;
        QTimer *timer_;
        
        // store fully qualified node names
        std::unordered_map<std::string, std::shared_ptr<GetStateClient>> lifecycle_clients_;

        // store the state of the lifecycle nodes to be able to provide it immediately by request, future development of CLI tool
        std::unordered_map<std::string, LifecycleState> lifecycle_node_states_;

        std::unordered_map<uint8_t, std::string> state_to_color_ = {
            {0, "#FF3838"}, // red
            {1, "#FFB302"}, // orange
            {2, "#CCCCCC"}, // grey
            {3, "#008000"}, // green
            {4, "#000000"}, // black
            {10, "#FFD700"}, // gold
            {11, "#B5651D"}, // brown
            {12, "#4B0082"}, // indigo
            {13, "#32CD32"}, // lime green
            {14, "#808080"}, // dark grey
            {15, "#FF4500"}, // orange red
        };
        const std::string default_color_ = "#000000"; // black
           

        std::shared_ptr<std::thread> monitoring_thread_;
        std::shared_ptr<std::thread> update_ui_thread_;
        std::shared_ptr<std::thread> spinner_thred_;
        const std::chrono::milliseconds monitoring_interval_ = std::chrono::milliseconds(5000);
        const std::chrono::milliseconds update_ui_interval_ = std::chrono::milliseconds(1000);
        std::mutex lifecycle_node_states_mutex_;
    };
}

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_lifecycle_plugin::RvizLifecyclePlugin, rviz_common::Panel)