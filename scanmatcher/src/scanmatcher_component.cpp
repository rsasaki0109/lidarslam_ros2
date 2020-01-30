#include "scanmatcher/scanmatcher_component.h"
#include <chrono>
using namespace std::chrono_literals;

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

        //TODO

    }   

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(graphslam::ScanMatcherComponent)