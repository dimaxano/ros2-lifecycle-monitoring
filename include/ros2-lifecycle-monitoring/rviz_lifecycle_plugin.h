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
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/list_hardware_components.hpp"
#include "controller_manager_msgs/srv/list_hardware_interfaces.hpp"

namespace rviz_lifecycle_plugin
{
    typedef typename rclcpp::Client<lifecycle_msgs::srv::GetState> GetStateClient;
    typedef typename rclcpp::Client<controller_manager_msgs::srv::ListControllers> ListControllerClient;
    typedef typename rclcpp::Client<controller_manager_msgs::srv::ListHardwareComponents> ListHardwareComponentsClient;
    typedef typename lifecycle_msgs::msg::State LifecycleState;
    typedef typename controller_manager_msgs::msg::ControllerState ControllerState;
    typedef typename controller_manager_msgs::msg::HardwareComponentState HardwareComponentsState;

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
        *  Requests the status of the controllers from the controller manager
        */
        void request_controller_state(const std::string& node_name);
        /*
        *  Requests the status of the hardware components from the controller manager
        */
        void request_hardware_state(const std::string& node_name);


        /*
        *  Updates the table widget at given row. If row does not exist, a new row is created
        */
        void update_table_widget(const size_t row, const std::string& node_name, const LifecycleState& state);
        void update_table_widget(const size_t row, const ControllerState& state);
        void update_table_widget(const size_t row, const HardwareComponentsState& state);


        /*
        *  Set color of the label according to the state of the lifecycle node
        */
        void set_label_color(QLabel* label, const LifecycleState& state);
        void set_label_color(QLabel* label, const ControllerState& state);
        void set_label_color(QLabel* label, const HardwareComponentsState& state);

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
        std::unordered_map<std::string, std::shared_ptr<ListControllerClient>> controller_clients_;
        std::unordered_map<std::string, std::shared_ptr<ListHardwareComponentsClient>> hw_components_clients_;

        // store the state of the lifecycle nodes to be able to provide it immediately by request, future development of CLI tool
        std::unordered_map<std::string, LifecycleState> lifecycle_node_states_;
        std::unordered_map<std::string, std::vector<ControllerState>> controller_states_;
        std::unordered_map<std::string, std::vector<HardwareComponentsState>> hw_components_states_;

        std::unordered_map<uint8_t, std::string> state_to_color_ = {
            {0, "#FF3838"}, // red - unknown state
            {1, "#FFB302"}, // orange - unconfigured
            {2, "#CCCCCC"}, // grey - inactive
            {3, "#008000"}, // green - active
            {4, "#000000"}, // black - finalized
            {10, "#FFD700"}, // gold - configuring
            {11, "#B5651D"}, // brown - cleaning up
            {12, "#4B0082"}, // indigo - shutting down
            {13, "#32CD32"}, // lime green - activating
            {14, "#808080"}, // dark grey - deactivating
            {15, "#FF4500"}, // orange red - error processing
        };
        std::unordered_map<std::string, std::string> controller_state_to_color_ = {
            {"inactive", "#FF3838"}, // red
            {"active", "#008000"}, // green
            {"unconfigured", "#FFB302"}, // orange
            {"finalized", "#000000"}, // black
        };
        const std::string default_color_ = "#000000"; // black


        std::shared_ptr<std::thread> monitoring_thread_;
        std::shared_ptr<std::thread> update_ui_thread_;
        std::shared_ptr<std::thread> spinner_thred_;
        const std::chrono::milliseconds monitoring_interval_ = std::chrono::milliseconds(5000);
        const std::chrono::milliseconds update_ui_interval_ = std::chrono::milliseconds(1000);
        std::mutex lifecycle_node_states_mutex_;
        std::mutex controller_states_mutex_;
        std::mutex hw_components_states_mutex_;
    };
}

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_lifecycle_plugin::RvizLifecyclePlugin, rviz_common::Panel)
