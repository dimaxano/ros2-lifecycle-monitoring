#include <iostream>
#include <sstream>

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include "ros2-lifecycle-monitoring/rviz_lifecycle_plugin.h"

namespace rviz_lifecycle_plugin
{
    RvizLifecyclePlugin::RvizLifecyclePlugin(QWidget *parent): Panel(parent) {
        utility_node_ = rclcpp::Node::make_shared("rviz_lifecycle_plugin");

        main_layout_ = new QVBoxLayout(this);
        nodes_states_table_ = new QTableWidget(0, 2, this);
        scroll_area_ = new QScrollArea(this);

        lifecycle_nodes_names_ = {};
        get_lifecycle_node_names(lifecycle_nodes_names_);

        size_t idx = 0;
        for(const auto& fully_qualified_node_name : lifecycle_nodes_names_){
            add_client(fully_qualified_node_name);

            auto state = LifecycleState();
            update_table_widget(idx, fully_qualified_node_name, state);
        
            idx++;
        }

        scroll_area_->setWidget(nodes_states_table_);
        scroll_area_->setWidgetResizable(true);

        main_layout_->addWidget(scroll_area_);

        this->setLayout(main_layout_);
    }

    RvizLifecyclePlugin::~RvizLifecyclePlugin() {}

    void RvizLifecyclePlugin::get_node_name_and_namespace(
        const std::string& fully_qualified_name,
        std::string& name, 
        std::string& name_space){

        auto pos = fully_qualified_name.rfind("/");
        if(pos != std::string::npos){
            name = fully_qualified_name.substr(pos+1, std::string::npos);
            name_space = fully_qualified_name.substr(0, pos);
        } else {
            name = fully_qualified_name.substr(0, 1);
        }
    }

    void RvizLifecyclePlugin::get_lifecycle_node_names(std::vector<std::string>& lifecycle_node_names){
        auto node_names = utility_node_->get_node_names();
        for(auto& fully_qualified_name : node_names){

            std::string node_name;
            std::string node_namespace;
            get_node_name_and_namespace(fully_qualified_name, node_name, node_namespace);

            auto services_names_types = utility_node_->get_service_names_and_types_by_node(node_name, node_namespace);
            
            bool lifecycle_found = false;
            for(const auto& kv : services_names_types){
                for(const auto& type_name : kv.second){
                    if(type_name.find("lifecycle") != std::string::npos){
                        lifecycle_node_names.push_back(fully_qualified_name);
                        lifecycle_found = true;
                        break;
                    }
                }
                if(lifecycle_found) break;
            }
        }
    }

    void RvizLifecyclePlugin::request_lifecycle_node_state(const std::string& fully_qualified_name){
        auto client = clients_[fully_qualified_name];
        auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
        
        client->async_send_request(request, [this, fully_qualified_name](GetStateClient::SharedFuture response){
                std::lock_guard<std::mutex> lock(lifecycle_node_states_mutex_);
                lifecycle_node_states_[fully_qualified_name] = response.get()->current_state;
            });
    }

    void RvizLifecyclePlugin::update_table_widget(const size_t row, const std::string& node_name, const LifecycleState& state) {
        if(row >= (size_t)nodes_states_table_->rowCount()){
            nodes_states_table_->insertRow(row);

            nodes_states_table_->setCellWidget(row, 0, new QLabel(QString::fromStdString(node_name)));

            auto node_status_label = new QLabel(QString::fromStdString(state.label));
            set_label_color(node_status_label, state);

            nodes_states_table_->setCellWidget(row, 1, node_status_label);
        } else {
            auto node_status_label = dynamic_cast<QLabel*>(nodes_states_table_->cellWidget(row, 1));
            if(node_status_label && node_status_label->text().toStdString() != state.label){
                node_status_label->setText(QString::fromStdString(state.label));
                
                set_label_color(node_status_label, state);
            }
        }
    }

    void RvizLifecyclePlugin::set_label_color(QLabel* label, const LifecycleState& state){
        std::stringstream ss;
        if(state_to_color_.find(state.id) != state_to_color_.end()){
            ss << "QLabel { color: " << state_to_color_[state.id] << ";}";
            label->setStyleSheet(ss.str().c_str());
        } 
        else {
            ss << "QLabel { color: " << default_color_ << ";}";
            label->setStyleSheet(ss.str().c_str());
        }
    }

    void RvizLifecyclePlugin::add_client(const std::string& fully_qualified_node_name){
        if(clients_.find(fully_qualified_node_name) == clients_.end()){
            std::string node_name;
            std::string node_namespace;
            get_node_name_and_namespace(fully_qualified_node_name, node_name, node_namespace);

            std::string service_name = fully_qualified_node_name + "/get_state";
            clients_[fully_qualified_node_name] = utility_node_->create_client<lifecycle_msgs::srv::GetState>(service_name);
        }
    }

    void RvizLifecyclePlugin::onInitialize() {
        rviz_common::Panel::onInitialize();

        spinner_thred_ = std::make_shared<std::thread>([this](){
            rclcpp::executors::MultiThreadedExecutor executor;
            executor.add_node(utility_node_);
            executor.spin();
        });

        monitoring_thread_ = std::make_shared<std::thread>(&RvizLifecyclePlugin::monitoring, this);

        update_ui_thread_ = std::make_shared<std::thread>(&RvizLifecyclePlugin::update_ui, this);
    }

    void RvizLifecyclePlugin::monitoring() {
        while(true){
            for(const auto& node_name : lifecycle_nodes_names_) {
                request_lifecycle_node_state(node_name);
            }
            std::this_thread::sleep_for(monitoring_interval_);
        }
    }

    void RvizLifecyclePlugin::update_ui() {
        while(true){
            {
                std::lock_guard<std::mutex> lock(lifecycle_node_states_mutex_);
                          
                size_t idx = 0;
                for(const auto& kv : lifecycle_node_states_){
                    auto fully_qualified_node_name = kv.first;
                    auto state = kv.second;

                    std::string node_name;
                    std::string node_namespace;
                    get_node_name_and_namespace(fully_qualified_node_name, node_name, node_namespace);

                    update_table_widget(idx, node_name, state);

                    idx++;
                }
                
            }

            std::this_thread::sleep_for(update_ui_interval_);
        }
    }
}
