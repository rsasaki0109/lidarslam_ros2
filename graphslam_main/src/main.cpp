#include <scanmatcher/scanmatcher_component.h>
#include <graph_based_slam/graph_based_slam_component.h>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
    std::cout << "main start" << std::endl;
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;

    rclcpp::executors::SingleThreadedExecutor exec;

    auto scanmatcher = std::make_shared<graphslam::ScanMatcherComponent>(options);
    exec.add_node(scanmatcher);
    auto graphbasedslam = std::make_shared<graphslam::GraphBasedSlamComponent>(options);
    exec.add_node(graphbasedslam);

    exec.spin();
    rclcpp::shutdown();

    return 0;
}