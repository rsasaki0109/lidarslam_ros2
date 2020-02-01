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
        listener_(tfbuffer_),
        broadcaster_(this)
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
        declare_parameter("trans_for_mapupdate", 1.5);
        get_parameter("trans_for_mapupdate", trans_for_mapupdate_);

        voxelgrid_.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);

        if(registration_method == "NDT"){
            pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt(new pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
            ndt->setResolution(ndt_resolution);
            registration_ = ndt;
            
        }
        else{
            pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>::Ptr gicp;
            registration_ = gicp;
        }

        initializePubSub();
        RCLCPP_INFO(get_logger(), "initialization end");
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
            RCLCPP_INFO(get_logger(), "initial_pose is received");
            std::cout << "initial_pose is received" << std::endl;
            corrent_pose_stamped_ = *msg;
            previous_position_ = corrent_pose_stamped_.pose.position;
            initial_pose_received_ = true;
        };

        auto cloud_callback =
        [this](const typename sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void
        {
            if(initial_pose_received_)
            {
                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
                pcl::fromROSMsg(*msg,*cloud_ptr);
                if(!initial_cloud_received_)
                {
                    RCLCPP_INFO(get_logger(), "create a first map");
                    std::cout << "create a first map" << std::endl;

                    initial_cloud_received_ = true;

                    Eigen::Matrix4f sim_trans = getSimTrans(corrent_pose_stamped_);
                    
                    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
                    pcl::transformPointCloud(*cloud_ptr, *transformed_cloud_ptr, sim_trans);
                    map_ += *transformed_cloud_ptr;
            

                    pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(map_));
                    registration_->setInputTarget(map_ptr);

                    sensor_msgs::msg::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::msg::PointCloud2);
                    pcl::toROSMsg(*map_ptr, *map_msg_ptr);
                    map_msg_ptr->header.frame_id = "map";
                    map_pub_->publish(*map_msg_ptr);
                    return;
                }

                if(initial_cloud_received_) receiveCloud(cloud_ptr, msg->header.stamp);
            }

        };

        initial_pose_sub_ = 
            create_subscription<geometry_msgs::msg::PoseStamped>(
                "initial_pose", rclcpp::SystemDefaultsQoS(), initial_pose_callback);  
    
        sub_input_cloud_ = 
            create_subscription<sensor_msgs::msg::PointCloud2>(
                "input_cloud", rclcpp::SensorDataQoS(), cloud_callback);    
        // pub
        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("current_pose", rclcpp::SystemDefaultsQoS());
        map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("map", rclcpp::SystemDefaultsQoS()); 
    }

    void ScanMatcherComponent::receiveCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr, rclcpp::Time stamp){
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
        voxelgrid_.setInputCloud(cloud_ptr);
        voxelgrid_.filter(*filtered_cloud_ptr);
        registration_->setInputSource(filtered_cloud_ptr);

        Eigen::Matrix4f sim_trans = getSimTrans(corrent_pose_stamped_);
        pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);

        registration_->align(*output_cloud, sim_trans);

        Eigen::Matrix4f final_transformation = registration_->getFinalTransformation();

        publishMapAndPose(cloud_ptr, final_transformation, stamp);

        std::cout << "---------------------------------------------------------" << std::endl;
        std::cout << "trans: " << trans_ << std::endl;
        std::cout << "number of filtered cloud points: " << filtered_cloud_ptr->size() << std::endl;
        std::cout << "number of map　points: " << map_.size() << std::endl;
        std::cout << "initial transformation:" << std::endl;
        std::cout <<  sim_trans << std::endl;
        std::cout << "has converged: " << registration_->hasConverged() << std::endl;
        std::cout << "fitness score: " << registration_->getFitnessScore() << std::endl;
        std::cout << "final transformation:" << std::endl;
        std::cout <<  final_transformation << std::endl;
        std::cout << "---------------------------------------------------------" << std::endl;

    }

    void ScanMatcherComponent::publishMapAndPose(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr, Eigen::Matrix4f final_transformation, rclcpp::Time stamp){
        tf2::Matrix3x3 rotation_matrix;
        rotation_matrix.setValue(static_cast<double>(final_transformation(0, 0)), static_cast<double>(final_transformation(0, 1)),
                       static_cast<double>(final_transformation(0, 2)), static_cast<double>(final_transformation(1, 0)),
                       static_cast<double>(final_transformation(1, 1)), static_cast<double>(final_transformation(1, 2)),
                       static_cast<double>(final_transformation(2, 0)), static_cast<double>(final_transformation(2, 1)),
                       static_cast<double>(final_transformation(2, 2)));

        geometry_msgs::msg::Vector3 vec;
        vec.x = static_cast<double>(final_transformation(0, 3));
        vec.y = static_cast<double>(final_transformation(1, 3));
        vec.z = static_cast<double>(final_transformation(2, 3));
        tf2::Quaternion quat_tf2;
        rotation_matrix.getRotation(quat_tf2);
        geometry_msgs::msg::Quaternion quat;
        quat.x = quat_tf2.x();
        quat.y = quat_tf2.y();
        quat.z = quat_tf2.z();
        quat.w = quat_tf2.w();
        
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = stamp;
        transform_stamped.header.frame_id = "map";
        transform_stamped.child_frame_id = "base_link";
        transform_stamped.transform.translation = vec;
        transform_stamped.transform.rotation = quat;
        broadcaster_.sendTransform(transform_stamped);

        corrent_pose_stamped_.header.stamp = stamp;
        corrent_pose_stamped_.pose.position.x = vec.x;
        corrent_pose_stamped_.pose.position.y = vec.y;
        corrent_pose_stamped_.pose.position.z = vec.z;
        corrent_pose_stamped_.pose.orientation = quat;
        pose_pub_->publish(corrent_pose_stamped_);

        trans_ = sqrt(pow(vec.x - previous_position_.x, 2.0) 
                        + pow(vec.y - previous_position_.y, 2.0) 
                        + pow(vec.z - previous_position_.z, 2.0)) ;    
        if (trans_ >= trans_for_mapupdate_){
            RCLCPP_INFO(get_logger(), "map updte");
            std::cout << "map updte" << std::endl;
            pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(map_));
            pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
            previous_position_.x = vec.x;
            previous_position_.y = vec.y;
            previous_position_.z = vec.z;
            pcl::transformPointCloud(*cloud_ptr, *transformed_cloud_ptr, final_transformation);
            map_ += *transformed_cloud_ptr;
            registration_->setInputTarget(map_ptr);

            sensor_msgs::msg::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::msg::PointCloud2);
            pcl::toROSMsg(*map_ptr, *map_msg_ptr);
            map_pub_->publish(map_msg_ptr);
        }
    }

    Eigen::Matrix4f ScanMatcherComponent::getSimTrans(geometry_msgs::msg::PoseStamped pose_stamped){
        geometry_msgs::msg::Point pos = pose_stamped.pose.position;
        geometry_msgs::msg::Quaternion quat = pose_stamped.pose.orientation;
        Eigen::Translation3f translation(pos.x, pos.y, pos.z);
        //Eigen::Quaternionf rotation(quat.x, quat.y, quat.z, quat.w);
        Eigen::Quaternionf rotation(quat.w, quat.x, quat.y, quat.z);
        Eigen::Matrix4f sim_trans = (translation * rotation).matrix();

        return sim_trans;
    }


}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(graphslam::ScanMatcherComponent)