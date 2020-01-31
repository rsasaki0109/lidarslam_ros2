#include "scanmatcher/scanmatcher_component.h"
#include <chrono>

using namespace std::chrono_literals;
//using namespace std::placeholders;

namespace graphslam
{
    ScanMatcherComponent::ScanMatcherComponent(const rclcpp::NodeOptions & options)
    : Node("scan_matcher", options),
        clock_(RCL_ROS_TIME), 
        tfbuffer_(std::make_shared<rclcpp::Clock>(clock_)),
        listener_(tfbuffer_)
    {
        
        double voxel_leaf_size;
        std::string registration_method;
        double ndt_resolution;

        declare_parameter("voxel_leaf_size", 0.1);
        get_parameter("voxel_leaf_size", voxel_leaf_size);
        declare_parameter("registration_method","NDT");
        get_parameter("registration_method",registration_method);
        declare_parameter("ndt_resolution", 5.0);
        get_parameter("ndt_resolution", ndt_resolution);

        voxelgrid_->setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);

        if(registration_method == "NDT"){
            pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt;
            ndt->setResolution(ndt_resolution);
            registration_ = ndt;
            
        }
        else{
            pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>::Ptr gicp;
            registration_ = gicp;
        }

        initializePubSub();

    }   

    void ScanMatcherComponent::initializePubSub(){
        RCLCPP_INFO(get_logger(), "initialize Publishers and Subscribers");
        // sub
        auto initial_pose_callback =
        [this](const typename geometry_msgs::msg::PoseStamped::SharedPtr msg) -> void
        {
            if (msg->header.frame_id == global_frame_id_) {
                RCLCPP_WARN(get_logger(),"This initial_pose is not in the global frame");
                return;
            }
            corrent_pose_stamped_ = *msg;
            initial_pose_received_ = true;
        };

        auto cloud_callback =
        [this](const typename sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void
        {
            //TODO
            if(initial_pose_received_)
            {}
        };

        initial_pose_sub_ = 
            create_subscription<geometry_msgs::msg::PoseStamped>(
                "initialpose", rclcpp::SystemDefaultsQoS(), initial_pose_callback);  
    
        sub_input_cloud_ = 
            create_subscription<sensor_msgs::msg::PointCloud2>(
                "input_cloud", rclcpp::SensorDataQoS(), cloud_callback);    
        // pub
        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("curent_pose", rclcpp::SystemDefaultsQoS());
        map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("map", rclcpp::SystemDefaultsQoS()); 
    }


}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(graphslam::ScanMatcherComponent)