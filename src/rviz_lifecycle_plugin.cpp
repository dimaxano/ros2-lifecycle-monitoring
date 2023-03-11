#include <iostream>
#include <sstream>
#include "ros2-lifecycle-monitoring/rviz_lifecycle_plugin.h"

namespace rviz_lifecycle_plugin
{
    RvizLifecyclePlugin::RvizLifecyclePlugin(QWidget *parent) : 
        Panel(parent)
    {
        utility_node_ = rclcpp::Node::make_shared("rviz_lifecycle_plugin");

        // TODO: INITIALIZE nodename_qlabel_mapping_
        // this->rosbag_play_health_ = new QLabel(this);
        
        // this->rosbag_play_health_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

        // this->rosbag_play_health_->setText(rosbag_play_not_running_);

        QVBoxLayout *main_layout = new QVBoxLayout(this);
        QScrollArea *scrollArea = new QScrollArea(this);

        node_names_ = new QListWidget(this);

        std::vector<std::string> lifecycle_nodes = {};
        get_lifecycle_node_names(lifecycle_nodes);

        for(const auto& node_name : lifecycle_nodes){
            node_names_->addItem(QString::fromStdString(node_name));
        }

        scrollArea->setWidget(node_names_);
        scrollArea->setWidgetResizable(true);

        main_layout->addWidget(scrollArea);

        this->setLayout(main_layout);

        // QObject::connect(this->toggle_pause_button_, SIGNAL(clicked()), this, SLOT(on_toggle_paused_pressed()));
    }

    RvizLifecyclePlugin::~RvizLifecyclePlugin() {}

    void RvizLifecyclePlugin::get_lifecycle_node_names(std::vector<std::string>& lifecycle_node_names){
        auto node_names = utility_node_->get_node_names();
        for(auto& node_name : node_names){
            std::cerr << "Checking node " << node_name << std::endl;

            if(node_name == "/rviz_lifecycle_plugin") continue; 

            std::string n_name = "";
            std::string n_space = "";

            auto n = node_name.rfind("/");
            if(n != std::string::npos){
                n_name = node_name.substr(n+1, std::string::npos);
                n_space = node_name.substr(0, n);
            } else {
                n_name = node_name.erase(0, 1);
            }

            std::cerr << "Node name " << n_name << std::endl;
            std::cerr << "Namespace " << n_space << std::endl;

            auto services_names_types = utility_node_->get_service_names_and_types_by_node(n_name, n_space);
            
            bool lifecycle_found = false;
            for(const auto& kv : services_names_types){
                std::cerr << "services_name " << kv.first << std::endl;
                for(const auto& type_name : kv.second){
                    if(type_name.find("lifecycle") != std::string::npos){
                        lifecycle_node_names.push_back(node_name);
                        lifecycle_found = true;
                        break;
                    }
                }
                if(lifecycle_found) break;
            }
        }
    }

    void RvizLifecyclePlugin::get_lifecycle_nodes_statuses(const std::vector<std::string>& nodes_names) {
        
        // for(const auto& node_name : nodes_names){
            
        // }
        
    }

    void RvizLifecyclePlugin::onInitialize()
    {
        rviz_common::Panel::onInitialize();

        // this->lifecycle_get_state_client_ = std::make_shared<
        //     cmr_clients_utils::BasicServiceClient<lifecycle_msgs::srv::GetState>>("service_client_lifecycle_get_state", "amcl/get_state");
        // this->monitoring_thread_ = std::make_shared<std::thread>(&RvizLifecyclePlugin::monitoring, this);
    }

    void RvizLifecyclePlugin::monitoring()
    {
        while (true) {
            
            std::this_thread::sleep_for(std::chrono::seconds(5));
        }
    }
}
