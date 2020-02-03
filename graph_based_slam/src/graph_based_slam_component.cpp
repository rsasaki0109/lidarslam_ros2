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

        std::cout << "voxel_leaf_size[m]:" << voxel_leaf_size << std::endl;
        std::cout << "ndt_resolution[m]:" << ndt_resolution << std::endl;
        std::cout << "------------------" << std::endl;

        voxelgrid_.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);

        ndt_.setResolution(ndt_resolution);

        initializePubSub();
        RCLCPP_INFO(get_logger(), "initialization end");
    }   

    void GraphBasedSlamComponent::initializePubSub(){
        RCLCPP_INFO(get_logger(), "initialize Publishers and Subscribers");
    }
 
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(graphslam::GraphBasedSlamComponent)