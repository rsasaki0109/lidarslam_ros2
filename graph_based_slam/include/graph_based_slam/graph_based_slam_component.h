#ifndef GS_GBS_COMPONENT_H_INCLUDED
#define GS_GBS_COMPONENT_H_INCLUDED

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define GS_GBS_EXPORT __attribute__ ((dllexport))
    #define GS_GBS_IMPORT __attribute__ ((dllimport))
  #else
    #define GS_GBS_EXPORT __declspec(dllexport)
    #define GS_GBS_IMPORT __declspec(dllimport)
  #endif
  #ifdef GS_GBS_BUILDING_DLL
    #define GS_GBS_PUBLIC GS_GBS_EXPORT
  #else
    #define GS_GBS_PUBLIC GS_GBS_IMPORT
  #endif
  #define GS_GBS_PUBLIC_TYPE GS_GBS_PUBLIC
  #define GS_GBS_LOCAL
#else
  #define GS_GBS_EXPORT __attribute__ ((visibility("default")))
  #define GS_GBS_IMPORT
  #if __GNUC__ >= 4
    #define GS_GBS_PUBLIC __attribute__ ((visibility("default")))
    #define GS_GBS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define GS_GBS_PUBLIC
    #define GS_GBS_LOCAL
  #endif
  #define GS_GBS_PUBLIC_TYPE
#endif

#if __cplusplus
} // extern "C"
#endif

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <graphslam_ros2_msgs/msg/map_array.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>

namespace graphslam
{
    class GraphBasedSlamComponent: public rclcpp::Node
    {
    public:
        GS_GBS_PUBLIC
        explicit GraphBasedSlamComponent(const rclcpp::NodeOptions & options);
    private:
        rclcpp::Clock clock_;
        tf2_ros::Buffer tfbuffer_;
        tf2_ros::TransformListener listener_;
        tf2_ros::TransformBroadcaster broadcaster_;

        pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt_;
        pcl::VoxelGrid<pcl::PointXYZI> voxelgrid_;

        geometry_msgs::msg::PoseStamped corrent_pose_stamped_;
        graphslam_ros2_msgs::msg::MapArray map_array_msg_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
        rclcpp::Subscription<graphslam_ros2_msgs::msg::MapArray>::SharedPtr input_cloud_sub_;
        geometry_msgs::msg::PoseStamped modified_corrent_pose_stamped_;
        graphslam_ros2_msgs::msg::MapArray modified_map_array_msg_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr modified_pose_pub_;
        rclcpp::Publisher<graphslam_ros2_msgs::msg::MapArray>::SharedPtr modified_map_array_pub_;
        rclcpp::TimerBase::SharedPtr loop_detect_timer_;

        void initializePubSub();
        void detectLoop();
        void doPoseAdjustment();
        void publishMapAndPose();
    };
}

#endif  //GS_GBS_COMPONENT_H_INCLUDED
