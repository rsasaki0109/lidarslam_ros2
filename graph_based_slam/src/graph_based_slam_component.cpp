#include "graph_based_slam/graph_based_slam_component.h"
#include <chrono>

using namespace std::chrono_literals;

namespace graphslam
{
    GraphBasedSlamComponent::GraphBasedSlamComponent(const rclcpp::NodeOptions & options)
    : Node("graph_based_slam", options),
        clock_(RCL_ROS_TIME), 
        tfbuffer_(std::make_shared<rclcpp::Clock>(clock_)),
        listener_(tfbuffer_),
        broadcaster_(this)
    {
        double voxel_leaf_size;
        double ndt_resolution;

        declare_parameter("voxel_leaf_size", 0.2);
        get_parameter("voxel_leaf_size", voxel_leaf_size);
        declare_parameter("ndt_resolution", 5.0);
        get_parameter("ndt_resolution", ndt_resolution);
        declare_parameter("loop_detection_period", 1000);
        get_parameter("loop_detection_period", loop_detection_period_);
        declare_parameter("threshold_loop_clousure", 1.0);
        get_parameter("threshold_loop_clousure", threshold_loop_clousure_);
        declare_parameter("distance_loop_clousure", 4.5);
        get_parameter("distance_loop_clousure", distance_loop_clousure_);

        std::cout << "voxel_leaf_size[m]:" << voxel_leaf_size << std::endl;
        std::cout << "ndt_resolution[m]:" << ndt_resolution << std::endl;
        std::cout << "loop_detection_period[Hz]:" << loop_detection_period_ << std::endl;
        std::cout << "threshold_loop_clousure:" << threshold_loop_clousure_ << std::endl;
        std::cout << "distance_loop_clousure:" << distance_loop_clousure_ << std::endl;
        std::cout << "------------------" << std::endl;

        voxelgrid_.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);

        ndt_.setResolution(ndt_resolution);

        initializePubSub();
        RCLCPP_INFO(get_logger(), "initialization end");
    }   

    void GraphBasedSlamComponent::initializePubSub(){
        RCLCPP_INFO(get_logger(), "initialize Publishers and Subscribers");

        auto map_array_callback =
        [this](const typename graphslam_ros2_msgs::msg::MapArray::SharedPtr msg_ptr) -> void
        {
            map_array_msg_ = *msg_ptr;
        };

        map_array_sub_ = 
            create_subscription<graphslam_ros2_msgs::msg::MapArray>(
                "map_array", rclcpp::SystemDefaultsQoS(), map_array_callback);  

        std::chrono::milliseconds period(loop_detection_period_);
        loop_detect_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period), 
            std::bind(&GraphBasedSlamComponent::searchLoop, this)
            );

    }

    void GraphBasedSlamComponent::searchLoop()
    {
        graphslam_ros2_msgs::msg::SubMap latest_submap;
        latest_submap = map_array_msg_.submaps.back();
        pcl::PointCloud<pcl::PointXYZI>::Ptr latest_submap_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(latest_submap.cloud, *latest_submap_cloud_ptr);
        ndt_.setInputTarget(latest_submap_cloud_ptr);
        double latest_moving_distance = latest_submap.distance;
        for(auto submap : map_array_msg_.submaps){

            if(latest_moving_distance - submap.distance > distance_loop_clousure_){

                pcl::PointCloud<pcl::PointXYZI>::Ptr submap_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
                pcl::fromROSMsg(submap.cloud, *submap_cloud_ptr);
                ndt_.setInputSource(submap_cloud_ptr);

                pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
                ndt_.align(*output_cloud_ptr);

                double fitness_score = ndt_.getFitnessScore();
                if(fitness_score < threshold_loop_clousure_){
                    doPoseAdjustment();
                }


            }
        }
        


    }

    void GraphBasedSlamComponent::doPoseAdjustment(){
        std::cout << "doPoseAdjustment" << std::endl;
        //TODO
    }
 
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(graphslam::GraphBasedSlamComponent)