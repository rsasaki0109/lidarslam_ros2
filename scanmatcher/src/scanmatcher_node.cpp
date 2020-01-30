#include <rclcpp/rclcpp.hpp>
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;

    rclcpp::executors::SingleThreadedExecutor exec;

    exec.spin();
    rclcpp::shutdown();

    return 0;
}