#include <scanmatcher/scanmatcher_component.h>
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
        declare_parameter("resolution", 5.0);
    }   

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(graphslam::ScanMatcherComponent)