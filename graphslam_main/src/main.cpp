//#include "scanmatcher/scanmatcher_component.h"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;

    rclcpp::executors::SingleThreadedExecutor exec;

    //auto scanmatcher = std::make_shared<graphslam::ScanMatcherComponent>(options);
    //exec.add_node(scanmatcher);

    exec.spin();
    rclcpp::shutdown();

    return 0;
}