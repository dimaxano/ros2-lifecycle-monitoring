#include <iostream>
#include <sstream>
#include "ros2-lifecycle-monitoring/rviz_lifecycle_plugin.h"

namespace rviz_lifecycle_plugin
{
    RvizLifecyclePlugin::RvizLifecyclePlugin(QWidget *parent) : 
        Panel(parent)
    {
        utility_node_ = rclcpp::Node::make_shared("rviz_lifecycle_plugin");

        main_layout_ = new QVBoxLayout(this);
        node_names_ = new QTableWidget(1, 2, this);
        scroll_area_ = new QScrollArea(this);

        lifecycle_nodes_ = {};
        get_lifecycle_node_names(lifecycle_nodes_);

        for(const auto& node_name : lifecycle_nodes_){
            add_client(node_name);
        }

        size_t idx = 0;
        for(const auto& node_name : lifecycle_nodes_){
            auto state = get_lifecycle_node_state(node_name);

            lifecycle_node_states_[node_name] = state;

            update_table_widget(idx, node_name, state);
        
            idx++;
        }

        scroll_area_->setWidget(node_names_);
        scroll_area_->setWidgetResizable(true);

        main_layout_->addWidget(scroll_area_);

        this->setLayout(main_layout_);

        // QObject::connect(this->toggle_pause_button_, SIGNAL(clicked()), this, SLOT(on_toggle_paused_pressed()));
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

    LifecycleState RvizLifecyclePlugin::get_lifecycle_node_state(const std::string& fully_qualified_name){
        std::string node_name;
        std::string node_namespace;
        get_node_name_and_namespace(fully_qualified_name, node_name, node_namespace);

        auto client = clients_[node_name];
        auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
        
        auto response = client->send_request(request);
        if(response){
            return response->current_state;
        }
        else {
            return LifecycleState();
        }
    }

    void RvizLifecyclePlugin::update_table_widget(const size_t row, const std::string& node_name, const LifecycleState& state) {
        if((row + 1) > (size_t)node_names_->rowCount()){
            node_names_->insertRow(row); // TODO: first row is empty, this is a bug

            node_names_->setCellWidget(row, 0, new QLabel(QString::fromStdString(node_name)));
            node_names_->setCellWidget(row, 1, new QLabel(QString::fromStdString(state.label)));
        } else {
            // TODO: write QLabel update logic
        }

        
    }

    void RvizLifecyclePlugin::add_client(const std::string& fully_qualified_name){
        if(!clients_.contains(fully_qualified_name)){
            std::string node_name;
            std::string node_namespace;
            get_node_name_and_namespace(fully_qualified_name, node_name, node_namespace);

            std::string client_name = node_name + "_client";
            std::string service_name = fully_qualified_name + "/get_state";
            clients_[node_name] = std::make_shared<GetStateClient>(client_name, service_name);
        }
    }

    void RvizLifecyclePlugin::onInitialize()
    {
        rviz_common::Panel::onInitialize();

        // this->monitoring_thread_ = std::make_shared<std::thread>(&RvizLifecyclePlugin::monitoring, this);
    }

    // check node state every second and update UI
    void RvizLifecyclePlugin::monitoring()
    {
        while(true){
            size_t idx = 0;
            for(const auto& node_name : lifecycle_nodes_){
                auto state = get_lifecycle_node_state(node_name);

                std::stringstream ss;
                if(state.label != lifecycle_node_states_[node_name].label){
                    lifecycle_node_states_[node_name] = state;
                    ss << node_name << " - " << "\t\t" << state.label;

                    // node_names_->item(idx)->setText(QString::fromStdString(ss.str()));
                }

                idx++;
            }

            std::this_thread::sleep_for(monitoring_interval_);
        }
    }
}
