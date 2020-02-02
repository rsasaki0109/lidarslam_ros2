#ifndef GS_SM_COMPONENT_H_INCLUDED
#define GS_SM_COMPONENT_H_INCLUDED

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define GS_SM_EXPORT __attribute__ ((dllexport))
    #define GS_SM_IMPORT __attribute__ ((dllimport))
  #else
    #define GS_SM_EXPORT __declspec(dllexport)
    #define GS_SM_IMPORT __declspec(dllimport)
  #endif
  #ifdef GS_SM_BUILDING_DLL
    #define GS_SM_PUBLIC GS_SM_EXPORT
  #else
    #define GS_SM_PUBLIC GS_SM_IMPORT
  #endif
  #define GS_SM_PUBLIC_TYPE GS_SM_PUBLIC
  #define GS_SM_LOCAL
#else
  #define GS_SM_EXPORT __attribute__ ((visibility("default")))
  #define GS_SM_IMPORT
  #if __GNUC__ >= 4
    #define GS_SM_PUBLIC __attribute__ ((visibility("default")))
    #define GS_SM_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define GS_SM_PUBLIC
    #define GS_SM_LOCAL
  #endif
  #define GS_SM_PUBLIC_TYPE
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

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>

namespace graphslam
{
    class ScanMatcherComponent: public rclcpp::Node
    {
    public:
        GS_SM_PUBLIC
        explicit ScanMatcherComponent(const rclcpp::NodeOptions & options);
    private:
        rclcpp::Clock clock_;
        tf2_ros::Buffer tfbuffer_;
        tf2_ros::TransformListener listener_;
        tf2_ros::TransformBroadcaster broadcaster_;

        std::string global_frame_id_;

        pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr registration_;
        pcl::VoxelGrid<pcl::PointXYZI> voxelgrid_;

        geometry_msgs::msg::Point previous_position_;
        double trans_;

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr initial_pose_sub_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_input_cloud_;
        geometry_msgs::msg::PoseStamped corrent_pose_stamped_;
        pcl::PointCloud<pcl::PointXYZI> map_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;

        void initializePubSub();
        void receiveCloud(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& input_cloud, rclcpp::Time stamp);
        void publishMapAndPose(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud_ptr, Eigen::Matrix4f final_transformation, rclcpp::Time stamp);
        Eigen::Matrix4f getSimTrans(geometry_msgs::msg::PoseStamped pose_stamped);

        bool initial_pose_received_{false};
        bool initial_cloud_received_{false};
        double trans_for_mapupdate_;

    };
}

#endif  //GS_SM_COMPONENT_H_INCLUDED
