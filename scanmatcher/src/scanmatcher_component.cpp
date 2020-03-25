#include "scanmatcher/scanmatcher_component.h"
#include <chrono>

using namespace std::chrono_literals;

namespace graphslam
{
    ScanMatcherComponent::ScanMatcherComponent(const rclcpp::NodeOptions & options)
    : Node("scan_matcher", options),
        clock_(RCL_ROS_TIME), 
        tfbuffer_(std::make_shared<rclcpp::Clock>(clock_)),
        listener_(tfbuffer_),
        broadcaster_(this)
    {

        std::string registration_method;
        double ndt_resolution;
        int ndt_num_threads;

        declare_parameter("global_frame_id", "map");
        get_parameter("global_frame_id", global_frame_id_);
        
        declare_parameter("registration_method","NDT");
        get_parameter("registration_method",registration_method);
        declare_parameter("ndt_resolution", 5.0);
        get_parameter("ndt_resolution", ndt_resolution);
        declare_parameter("ndt_num_threads", 0);
        get_parameter("ndt_num_threads", ndt_num_threads);
        declare_parameter("trans_for_mapupdate", 1.5);
        get_parameter("trans_for_mapupdate", trans_for_mapupdate_);
        declare_parameter("vg_size_for_input", 0.2);
        get_parameter("vg_size_for_input", vg_size_for_input_);
        declare_parameter("vg_size_for_map", 0.1);
        get_parameter("vg_size_for_map", vg_size_for_map_);

        declare_parameter("use_odom", false);
        get_parameter("use_odom", use_odom_);
        declare_parameter("use_imu", false);
        get_parameter("use_imu", use_imu_);


        std::cout << "registration_method:" << registration_method << std::endl;
        std::cout << "ndt_resolution[m]:" << ndt_resolution << std::endl;
        std::cout << "ndt_num_threads:" << ndt_num_threads << std::endl;
        std::cout << "trans_for_mapupdate[m]:" << trans_for_mapupdate_ << std::endl;
        std::cout << "vg_size_for_input[m]:" << vg_size_for_input_ << std::endl;
        std::cout << "vg_size_for_map[m]:" << vg_size_for_map_ << std::endl;
        std::cout << "use_odom:" << std::boolalpha << use_odom_ << std::endl;
        std::cout << "use_imu:" << std::boolalpha << use_imu_ << std::endl;
        std::cout << "------------------" << std::endl;

        if(registration_method == "NDT"){
            
            pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt(new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
            ndt->setResolution(ndt_resolution);
            ndt->setTransformationEpsilon(0.01);

            //ndt_omp
            ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
            if(ndt_num_threads > 0) ndt->setNumThreads(ndt_num_threads);
            
            registration_ = ndt;
            
        }
        else{
            pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>::Ptr gicp(new pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>());
            registration_ = gicp;
        }

        map_.header.frame_id = global_frame_id_;
        map_array_msg_.header.frame_id = global_frame_id_;

        initializePubSub();
        RCLCPP_INFO(get_logger(), "initialization end");
    }   

    void ScanMatcherComponent::initializePubSub(){
        RCLCPP_INFO(get_logger(), "initialize Publishers and Subscribers");
        // sub
        auto initial_pose_callback =
        [this](const typename geometry_msgs::msg::PoseStamped::SharedPtr msg) -> void
        {
            if (msg->header.frame_id != global_frame_id_) {
                RCLCPP_WARN(get_logger(),"This initial_pose is not in the global frame");
                return;
            }
            std::cout << "initial_pose is received" << std::endl;

            corrent_pose_stamped_ = *msg;
            previous_position_.x() = corrent_pose_stamped_.pose.position.x;
            previous_position_.y() = corrent_pose_stamped_.pose.position.y;
            previous_position_.z() = corrent_pose_stamped_.pose.position.z;
            initial_pose_received_ = true;

            std::cout << "x:" << corrent_pose_stamped_.pose.position.x << ","
                      << "y:" << corrent_pose_stamped_.pose.position.y << ","
                      << "z:" << corrent_pose_stamped_.pose.position.z << std::endl;
            

            if(use_imu_){
                Eigen::Matrix4f sim_trans = getSimTrans(corrent_pose_stamped_);
                tf2::Matrix3x3 mat_trans_tf2;
                mat_trans_tf2.setValue(
                    static_cast<double>(sim_trans(0, 0)), static_cast<double>(sim_trans(0, 1)),
                    static_cast<double>(sim_trans(0, 2)), static_cast<double>(sim_trans(1, 0)),
                    static_cast<double>(sim_trans(1, 1)), static_cast<double>(sim_trans(1, 2)),
                    static_cast<double>(sim_trans(2, 0)), static_cast<double>(sim_trans(2, 1)),
                    static_cast<double>(sim_trans(2, 2)));

                double roll,pitch,yaw;
                mat_trans_tf2.getRPY(roll, pitch, yaw, 1);
                rollpitchyaw_(0) = roll;
                rollpitchyaw_(1) = pitch;
                rollpitchyaw_(2) = yaw;
            }
        };

        auto cloud_callback =
        [this](const typename sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void
        {
            if(initial_pose_received_)
            {
                sensor_msgs::msg::PointCloud2 transformerd_msg;
                try{
                    tf2::TimePoint time_point = tf2::TimePoint(
                        std::chrono::seconds(msg->header.stamp.sec) +
                        std::chrono::nanoseconds(msg->header.stamp.nanosec));
                    const geometry_msgs::msg::TransformStamped transform = tfbuffer_.lookupTransform(
                        "base_link", msg->header.frame_id, time_point);
                    tf2::doTransform(*msg, transformerd_msg, transform);//TODO:slow now(https://github.com/ros/geometry2/pull/432)
                }
                catch (tf2::TransformException& e){
                    RCLCPP_ERROR(this->get_logger(),"%s",e.what());
                    return;
                }

                pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_ptr(new pcl::PointCloud<pcl::PointXYZI>());
                pcl::fromROSMsg(transformerd_msg,*tmp_ptr);
                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
                voxelgrid_.setLeafSize(vg_size_for_map_, vg_size_for_map_, vg_size_for_map_);
                voxelgrid_.setInputCloud(tmp_ptr);
                voxelgrid_.filter(*cloud_ptr);

                if(!initial_cloud_received_)
                {
                    RCLCPP_INFO(get_logger(), "create a first map");

                    initial_cloud_received_ = true;

                    Eigen::Matrix4f sim_trans = getSimTrans(corrent_pose_stamped_);
                    
                    if(use_imu_){
                        tf2::Matrix3x3 mat_trans_tf2;
                        mat_trans_tf2.setValue(
                        static_cast<double>(sim_trans(0, 0)), static_cast<double>(sim_trans(0, 1)),
                        static_cast<double>(sim_trans(0, 2)), static_cast<double>(sim_trans(1, 0)),
                        static_cast<double>(sim_trans(1, 1)), static_cast<double>(sim_trans(1, 2)),
                        static_cast<double>(sim_trans(2, 0)), static_cast<double>(sim_trans(2, 1)),
                        static_cast<double>(sim_trans(2, 2)));

                        double roll,pitch,yaw;
                        mat_trans_tf2.getRPY(roll, pitch, yaw, 1);    

                        Eigen::Translation3f translation(sim_trans(0, 3), sim_trans(1, 3), sim_trans(2, 3));
                        Eigen::AngleAxisf rotation_x(rollpitchyaw_(0), Eigen::Vector3f::UnitX());
                        Eigen::AngleAxisf rotation_y(rollpitchyaw_(1), Eigen::Vector3f::UnitY());
                        Eigen::AngleAxisf rotation_z(yaw, Eigen::Vector3f::UnitZ());     

                        sim_trans = (translation * rotation_z * rotation_y * rotation_x).matrix() ;
                    }
                    
                    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
                    pcl::transformPointCloud(*cloud_ptr, *transformed_cloud_ptr, sim_trans);
                    registration_->setInputTarget(transformed_cloud_ptr);
                    
                    // map
                    map_ += *transformed_cloud_ptr;
                    sensor_msgs::msg::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::msg::PointCloud2);
                    pcl::toROSMsg(*transformed_cloud_ptr, *map_msg_ptr);

                    //map array
                    sensor_msgs::msg::PointCloud2::Ptr transformed_cloud_msg_ptr(new sensor_msgs::msg::PointCloud2);
                    pcl::toROSMsg(*transformed_cloud_ptr, *transformed_cloud_msg_ptr);
                    graphslam_ros2_msgs::msg::SubMap submap;
                    submap.header = msg->header;
                    submap.distance = 0;
                    submap.pose = corrent_pose_stamped_.pose;
                    submap.cloud = *transformed_cloud_msg_ptr;
                    map_array_msg_.header = msg->header;
                    map_array_msg_.submaps.push_back(submap);

                    map_pub_->publish(submap.cloud);

                }

                if(initial_cloud_received_) receiveCloud(cloud_ptr, msg->header.stamp);
            }

        };

        auto imu_callback =
        [this](const typename sensor_msgs::msg::Imu::SharedPtr msg) -> void
        {
            if(initial_pose_received_)
            {
                receiveImu(*msg); 
            }
        };

        auto odom_callback =
        [this](const typename nav_msgs::msg::Odometry::SharedPtr msg) -> void
        {
            if(initial_pose_received_)
            {
                receiveOdom(*msg); 
            }
        };

        initial_pose_sub_ = 
            create_subscription<geometry_msgs::msg::PoseStamped>(
                "initial_pose", rclcpp::SystemDefaultsQoS(), initial_pose_callback);  
    
        imu_sub_ = 
            create_subscription<sensor_msgs::msg::Imu>(
                "imu", rclcpp::SensorDataQoS(), imu_callback);    

        odom_sub_ = 
            create_subscription<nav_msgs::msg::Odometry>(
                "odom", rclcpp::SensorDataQoS(), odom_callback);  

        input_cloud_sub_ = 
            create_subscription<sensor_msgs::msg::PointCloud2>(
                "input_cloud", rclcpp::SensorDataQoS(), cloud_callback);
   
        // pub
        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("current_pose", rclcpp::SystemDefaultsQoS());
        map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("map", rclcpp::SystemDefaultsQoS()); 
        map_array_pub_ = create_publisher<graphslam_ros2_msgs::msg::MapArray>("map_array", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable()); 
        path_pub_ = create_publisher<nav_msgs::msg::Path>("path", rclcpp::SystemDefaultsQoS());
    }

    void ScanMatcherComponent::receiveCloud(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud_ptr, rclcpp::Time stamp){
        
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
        voxelgrid_.setLeafSize(vg_size_for_input_, vg_size_for_input_, vg_size_for_input_);
        voxelgrid_.setInputCloud(cloud_ptr);
        voxelgrid_.filter(*filtered_cloud_ptr);
        registration_->setInputSource(filtered_cloud_ptr);

        Eigen::Matrix4f sim_trans = getSimTrans(corrent_pose_stamped_);

        if(use_imu_){
            tf2::Matrix3x3 mat_trans_tf2;
            mat_trans_tf2.setValue(
                static_cast<double>(sim_trans(0, 0)), static_cast<double>(sim_trans(0, 1)),
                static_cast<double>(sim_trans(0, 2)), static_cast<double>(sim_trans(1, 0)),
                static_cast<double>(sim_trans(1, 1)), static_cast<double>(sim_trans(1, 2)),
                static_cast<double>(sim_trans(2, 0)), static_cast<double>(sim_trans(2, 1)),
                static_cast<double>(sim_trans(2, 2)));

            double roll,pitch,yaw;
            mat_trans_tf2.getRPY(roll, pitch, yaw, 1);  

            tf2::Quaternion quat_tf2;  
            quat_tf2.setRPY(rollpitchyaw_(0), rollpitchyaw_(1), yaw);  
            rollpitchyaw_(2) = yaw;

            Eigen::Translation3f translation(sim_trans(0, 3), sim_trans(1, 3), sim_trans(2, 3));
            Eigen::AngleAxisf rotation_x(rollpitchyaw_(0), Eigen::Vector3f::UnitX());
            Eigen::AngleAxisf rotation_y(rollpitchyaw_(1), Eigen::Vector3f::UnitY());
            Eigen::AngleAxisf rotation_z(yaw, Eigen::Vector3f::UnitZ());     

            sim_trans = (translation * rotation_z * rotation_y * rotation_x).matrix() ;

        }
        sim_trans = sim_trans ;
        pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);

        rclcpp::Clock system_clock;
        rclcpp::Time time_align_start = system_clock.now();
        registration_->align(*output_cloud, sim_trans);
        rclcpp::Time time_align_end = system_clock.now();

        Eigen::Matrix4f final_transformation = registration_->getFinalTransformation();

        publishMapAndPose(cloud_ptr, final_transformation, stamp);

        tf2::Matrix3x3 mat_tf2;
        mat_tf2.setValue(
            static_cast<double>(final_transformation(0, 0)), static_cast<double>(final_transformation(0, 1)),
            static_cast<double>(final_transformation(0, 2)), static_cast<double>(final_transformation(1, 0)),
            static_cast<double>(final_transformation(1, 1)), static_cast<double>(final_transformation(1, 2)),
            static_cast<double>(final_transformation(2, 0)), static_cast<double>(final_transformation(2, 1)),
            static_cast<double>(final_transformation(2, 2)));

        double roll,pitch,yaw;
        mat_tf2.getRPY(roll, pitch, yaw, 1);//mat2rpy

        std::cout << "---------------------------------------------------------" << std::endl;
        
        //std::cout << "nanoseconds: " << stamp.nanoseconds() << std::endl;
        //std::cout << "trans: " << trans_ << std::endl;
        
        std::cout << "align time:" << time_align_end.seconds() - time_align_start.seconds() << "s" << std::endl;
        
        std::cout << "number of filtered cloud points: " << filtered_cloud_ptr->size() << std::endl;
        std::cout << "number of map　points: " << map_.size() << std::endl;
        std::cout << "initial transformation:" << std::endl;
        std::cout <<  sim_trans << std::endl;
        std::cout << "has converged: " << registration_->hasConverged() << std::endl;
        std::cout << "fitness score: " << registration_->getFitnessScore() << std::endl;
        std::cout << "final transformation:" << std::endl;
        std::cout <<  final_transformation << std::endl;
        std::cout << "rpy" << std::endl;
        std::cout << "roll:" << roll * 180 / M_PI << ","
                  << "pitch:" << pitch * 180 / M_PI << ","
                  << "yaw:" << yaw * 180 / M_PI << std::endl;
        
        int num_submaps = map_array_msg_.submaps.size();
        std::cout << "num_submaps:" << num_submaps << std::endl;
        std::cout << "moving distance:" << latest_distance_ << std::endl;
        std::cout << "---------------------------------------------------------" << std::endl;

    }

    void ScanMatcherComponent::publishMapAndPose(
        const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud_ptr, 
        Eigen::Matrix4f final_transformation, rclcpp::Time stamp){
        
        Eigen::Matrix4f pc_transformation = final_transformation;
        final_transformation = final_transformation ;
        Eigen::Vector3d vec;
        vec.x() = static_cast<double>(final_transformation(0, 3));
        vec.y() = static_cast<double>(final_transformation(1, 3));
        vec.z() = static_cast<double>(final_transformation(2, 3));

        Eigen::Matrix3d rot_mat = final_transformation.block<3, 3>(0, 0).cast<double>();
        Eigen::Quaterniond quat_eig(rot_mat);
        geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(quat_eig);
        
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = stamp;
        transform_stamped.header.frame_id = "map";
        transform_stamped.child_frame_id = "base_link";
        transform_stamped.transform.translation.x = vec.x();
        transform_stamped.transform.translation.y = vec.y();
        transform_stamped.transform.translation.z = vec.z();
        transform_stamped.transform.rotation = quat_msg;
        broadcaster_.sendTransform(transform_stamped);

        corrent_pose_stamped_.header.stamp = stamp;
        corrent_pose_stamped_.pose.position.x = vec.x();
        corrent_pose_stamped_.pose.position.y = vec.y();
        corrent_pose_stamped_.pose.position.z = vec.z();
        corrent_pose_stamped_.pose.orientation = quat_msg;
        pose_pub_->publish(corrent_pose_stamped_);

        trans_ = (vec - previous_position_).norm();  
        if (trans_ >= trans_for_mapupdate_){
            RCLCPP_INFO(get_logger(), "map update");

            pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
        
            previous_position_ = vec;
 
            //pcl::transformPointCloud(*cloud_ptr, *transformed_cloud_ptr, final_transformation);
            pcl::transformPointCloud(*cloud_ptr, *transformed_cloud_ptr, pc_transformation);

            map_ += *transformed_cloud_ptr;
            pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(map_));

            //registration_->setInputTarget(map_ptr);//TODO:change scan2scan matching to submap2scan matching
            registration_->setInputTarget(transformed_cloud_ptr);
        
            sensor_msgs::msg::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::msg::PointCloud2);
            pcl::toROSMsg(*map_ptr, *map_msg_ptr);
            map_msg_ptr->header.frame_id = "map";
            map_pub_->publish(map_msg_ptr);
            
            /*map array */
            sensor_msgs::msg::PointCloud2::Ptr transformed_cloud_msg_ptr(new sensor_msgs::msg::PointCloud2);
            pcl::toROSMsg(*transformed_cloud_ptr, *transformed_cloud_msg_ptr);
            //sensor_msgs::msg::PointCloud2::Ptr cloud_msg_ptr(new sensor_msgs::msg::PointCloud2);
            //pcl::toROSMsg(*cloud_ptr, *cloud_msg_ptr);

            graphslam_ros2_msgs::msg::SubMap submap;
            submap.header.frame_id = global_frame_id_;
            submap.header.stamp = corrent_pose_stamped_.header.stamp;
            latest_distance_ += trans_;
            submap.distance = latest_distance_;
            submap.pose = corrent_pose_stamped_.pose;
            submap.cloud = *transformed_cloud_msg_ptr;//TODO
            //submap.cloud = *cloud_msg_ptr;
            submap.cloud.header.frame_id = global_frame_id_;
            map_array_msg_.header.stamp = corrent_pose_stamped_.header.stamp;
            map_array_msg_.submaps.push_back(submap);
            map_array_pub_->publish(map_array_msg_);
            
            nav_msgs::msg::Path path;
            path.header.frame_id = "map";
            for(auto submap : map_array_msg_.submaps){
                geometry_msgs::msg::PoseStamped pose_stamped;
                pose_stamped.header = submap.header;
                pose_stamped.pose = submap.pose;
                path.poses.push_back(pose_stamped);
            }
            path_pub_->publish(path);

        }
    }

    Eigen::Matrix4f ScanMatcherComponent::getSimTrans(geometry_msgs::msg::PoseStamped pose_stamped){

        Eigen::Affine3d affine;
        tf2::fromMsg(pose_stamped.pose, affine);
        Eigen::Matrix4f sim_trans = affine.matrix().cast<float>();

        return sim_trans;
    }

    void ScanMatcherComponent::receiveImu(const sensor_msgs::msg::Imu imu_msg){

        if(!use_imu_) return;

        tf2::Quaternion previous_quat_tf;
        double roll, pitch, yaw;
        tf2::fromMsg(imu_msg.orientation, previous_quat_tf);
        tf2::Matrix3x3(previous_quat_tf).getRPY(roll, pitch, yaw);
        rollpitchyaw_(0) = roll;
        rollpitchyaw_(1) = pitch;
        rollpitchyaw_(2) = yaw;

    }

    void ScanMatcherComponent::receiveOdom(const nav_msgs::msg::Odometry odom_msg){

        if(!use_odom_) return;

        double current_time_odom = odom_msg.header.stamp.sec 
                                    + odom_msg.header.stamp.nanosec * 1e-9;
        if(previous_time_odom_ == -1){
            previous_time_odom_ = current_time_odom;
            return;
        }
        double dt_odom = current_time_odom - previous_time_odom_;

        previous_time_odom_ = current_time_odom; 

        tf2::Quaternion previous_quat_tf;
        double roll, pitch, yaw;
        tf2::fromMsg(corrent_pose_stamped_.pose.orientation, previous_quat_tf);
        tf2::Matrix3x3(previous_quat_tf).getRPY(roll, pitch, yaw);

        roll += odom_msg.twist.twist.angular.x * dt_odom;
        pitch += odom_msg.twist.twist.angular.y * dt_odom;
        yaw += odom_msg.twist.twist.angular.z * dt_odom;

        Eigen::Quaterniond quat_eig = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
                    * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
        
        geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(quat_eig);
        
        Eigen::Vector3d odom{odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y, odom_msg.twist.twist.linear.z};
        Eigen::Vector3d delta_position = quat_eig.matrix() * dt_odom * odom;

        corrent_pose_stamped_.pose.position.x += delta_position.x();
        corrent_pose_stamped_.pose.position.y += delta_position.y();
        corrent_pose_stamped_.pose.position.z += delta_position.z();
        corrent_pose_stamped_.pose.orientation = quat_msg;

        return;
    }

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(graphslam::ScanMatcherComponent)
