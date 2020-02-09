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
        declare_parameter("use_imu", false);
        get_parameter("use_imu", use_imu_);
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
        declare_parameter("vg_size_for_viz", 0.1);
        get_parameter("vg_size_for_viz", vg_size_for_viz_);

        std::cout << "registration_method:" << registration_method << std::endl;
        std::cout << "ndt_resolution[m]:" << ndt_resolution << std::endl;
        std::cout << "ndt_num_threads:" << ndt_num_threads << std::endl;
        std::cout << "trans_for_mapupdate[m]:" << trans_for_mapupdate_ << std::endl;
        std::cout << "vg_size_for_input[m]:" << vg_size_for_input_ << std::endl;
        std::cout << "vg_size_for_map[m]:" << vg_size_for_map_ << std::endl;
        std::cout << "vg_size_for_viz[m]:" << vg_size_for_viz_ << std::endl;
        std::cout << "use_imu:" << std::boolalpha << use_imu_ << std::endl;
        std::cout << "------------------" << std::endl;

        if(registration_method == "NDT"){
            //pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt(new pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
            pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt(new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
            ndt->setResolution(ndt_resolution);

            //ndt_omp
            ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
            if(ndt_num_threads > 0) ndt->setNumThreads(ndt_num_threads);
            
            registration_ = ndt;
            
        }
        else{
            pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>::Ptr gicp;
            registration_ = gicp;
        }

        map_.header.frame_id = "map";
        map_array_msg_.header.frame_id = "map";

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
            //RCLCPP_INFO(get_logger(), "initial_pose is received");
            std::cout << "initial_pose is received" << std::endl;

            corrent_pose_stamped_ = *msg;
            previous_position_.x() = corrent_pose_stamped_.pose.position.x;
            previous_position_.y() = corrent_pose_stamped_.pose.position.y;
            previous_position_.z() = corrent_pose_stamped_.pose.position.z;
            initial_pose_received_ = true;
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
                    initial_pos_mat_ = sim_trans;
                    
                    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
                    pcl::transformPointCloud(*cloud_ptr, *transformed_cloud_ptr, sim_trans);
                    registration_->setInputTarget(transformed_cloud_ptr);
                    
                    // map
                    map_ += *transformed_cloud_ptr;
                    sensor_msgs::msg::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::msg::PointCloud2);
                    pcl::toROSMsg(*transformed_cloud_ptr, *map_msg_ptr);
                    map_pub_->publish(*map_msg_ptr);

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
                }

                if(initial_cloud_received_) receiveCloud(cloud_ptr, msg->header.stamp);
            }

        };

        auto imu_callback =
        [this](const typename sensor_msgs::msg::Imu::SharedPtr msg) -> void
        {
            if(use_imu_ && initial_pose_received_)
            {
                receiveImu(*msg); 
            }
        };


        initial_pose_sub_ = 
            create_subscription<geometry_msgs::msg::PoseStamped>(
                "initial_pose", rclcpp::SystemDefaultsQoS(), initial_pose_callback);  
    
        imu_sub_ = 
            create_subscription<sensor_msgs::msg::Imu>(
                "imu", rclcpp::SensorDataQoS(), imu_callback);    

        input_cloud_sub_ = 
            create_subscription<sensor_msgs::msg::PointCloud2>(
                "input_cloud", rclcpp::SensorDataQoS(), cloud_callback);
   
        // pub
        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("current_pose", rclcpp::SystemDefaultsQoS());
        map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("map", rclcpp::SystemDefaultsQoS()); 
        map_array_pub_ = create_publisher<graphslam_ros2_msgs::msg::MapArray>("map_array", rclcpp::SystemDefaultsQoS()); 
    }

    void ScanMatcherComponent::receiveCloud(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud_ptr, rclcpp::Time stamp){
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
        voxelgrid_.setLeafSize(vg_size_for_input_, vg_size_for_input_, vg_size_for_input_);
        voxelgrid_.setInputCloud(cloud_ptr);
        voxelgrid_.filter(*filtered_cloud_ptr);
        registration_->setInputSource(filtered_cloud_ptr);

        Eigen::Matrix4f sim_trans = getSimTrans(corrent_pose_stamped_);
        sim_trans = sim_trans * initial_pos_mat_.inverse();
        pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);

        rclcpp::Clock system_clock;
        rclcpp::Time time_align_start = system_clock.now();
        registration_->align(*output_cloud, sim_trans);
        rclcpp::Time time_align_end = system_clock.now();

        Eigen::Matrix4f final_transformation = registration_->getFinalTransformation();

        publishMapAndPose(cloud_ptr, final_transformation, stamp);

        std::cout << "---------------------------------------------------------" << std::endl;
        std::cout << "nanoseconds: " << stamp.nanoseconds() << std::endl;
        std::cout << "trans: " << trans_ << std::endl;
        std::cout << "align time:" << time_align_end.seconds() - time_align_start.seconds() << "s" << std::endl;
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

    void ScanMatcherComponent::publishMapAndPose(
        const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud_ptr, 
        Eigen::Matrix4f final_transformation, rclcpp::Time stamp){
        
        Eigen::Matrix4f pc_transformation = final_transformation;
        final_transformation = final_transformation * initial_pos_mat_;

        Eigen::Vector3d vec;
        vec.x() = static_cast<double>(final_transformation(0, 3));
        vec.y() = static_cast<double>(final_transformation(1, 3));
        vec.z() = static_cast<double>(final_transformation(2, 3));

        Eigen::Matrix3d rot_mat = final_transformation.block<3, 3>(0, 0).cast<double>();
        Eigen::Quaterniond q_eig(rot_mat);
        geometry_msgs::msg::Quaternion quat = tf2::toMsg(q_eig);
        
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = stamp;
        transform_stamped.header.frame_id = "map";
        transform_stamped.child_frame_id = "base_link";
        transform_stamped.transform.translation.x = vec.x();
        transform_stamped.transform.translation.y = vec.y();
        transform_stamped.transform.translation.z = vec.z();
        transform_stamped.transform.rotation = quat;
        broadcaster_.sendTransform(transform_stamped);

        corrent_pose_stamped_.header.stamp = stamp;
        corrent_pose_stamped_.pose.position.x = vec.x();
        corrent_pose_stamped_.pose.position.y = vec.y();
        corrent_pose_stamped_.pose.position.z = vec.z();
        corrent_pose_stamped_.pose.orientation = quat;
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

            registration_->setInputTarget(map_ptr);//TODO
            
            pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_map_ptr(new pcl::PointCloud<pcl::PointXYZI>());
            voxelgrid_.setLeafSize(vg_size_for_viz_, vg_size_for_viz_, vg_size_for_viz_);
            voxelgrid_.setInputCloud(map_ptr);
            voxelgrid_.filter(*filtered_map_ptr);

            sensor_msgs::msg::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::msg::PointCloud2);
            //pcl::toROSMsg(*map_ptr, *map_msg_ptr);
            pcl::toROSMsg(*filtered_map_ptr, *map_msg_ptr);
            map_msg_ptr->header.frame_id = "map";
            map_pub_->publish(map_msg_ptr);

            //TODO:change map_ to map_array
            //map array
            /*
            sensor_msgs::msg::PointCloud2::Ptr transformed_cloud_msg_ptr(new sensor_msgs::msg::PointCloud2);
            pcl::toROSMsg(*transformed_cloud_ptr, *transformed_cloud_msg_ptr);
            graphslam_ros2_msgs::msg::SubMap submap;
            submap.header = corrent_pose_stamped_.header;
            submap.distance = trans_ + map_array_msg_.submaps.end()->distance;
            submap.pose = corrent_pose_stamped_.pose;
            submap.cloud = *transformed_cloud_msg_ptr;
            map_array_msg_.header = corrent_pose_stamped_.header;
            map_array_msg_.submaps.push_back(submap);
            map_array_pub_->publish(map_array_msg_);
            */

        }
    }

    Eigen::Matrix4f ScanMatcherComponent::getSimTrans(geometry_msgs::msg::PoseStamped pose_stamped){
        geometry_msgs::msg::Point pos = pose_stamped.pose.position;
        geometry_msgs::msg::Quaternion quat = pose_stamped.pose.orientation;
        Eigen::Translation3f translation(pos.x, pos.y, pos.z);
        Eigen::Quaternionf rotation(quat.w, quat.x, quat.y, quat.z);
        Eigen::Matrix4f sim_trans = (translation * rotation).matrix();

        return sim_trans;
    }

    void ScanMatcherComponent::receiveImu(const sensor_msgs::msg::Imu imu_msg){
        //TODO: not working well
        // gravity
        double alpha = 0.8;//a low-pass filter parameter
        Eigen::Vector3d gravity;
        gravity.x() = alpha * gravity_.x() + (1 - alpha) * imu_msg.linear_acceleration.x;
        gravity.y() = alpha * gravity_.y() + (1 - alpha) * imu_msg.linear_acceleration.y;
        gravity.z() = alpha * gravity_.z() + (1 - alpha) * imu_msg.linear_acceleration.z;
        double d_gravity = (gravity - gravity_).norm();
        gravity_.x() = gravity.x();
        gravity_.y() = gravity.y();
        gravity_.z() = gravity.z();

        if(d_gravity > 1.0) return;

        // predict 
        current_stamp_ = imu_msg.header.stamp;

        // dt_imu
        double current_time_imu = imu_msg.header.stamp.sec 
                                    + imu_msg.header.stamp.nanosec * 1e-9;
        if(previous_time_imu_ == -1){
            previous_time_imu_ = current_time_imu;
            return;
        }
        double dt_imu = current_time_imu - previous_time_imu_;
        previous_time_imu_ = current_time_imu;
        
        // state
        geometry_msgs::msg::Point pos_msg = corrent_pose_stamped_.pose.position;
        Eigen::Vector3d pos(pos_msg.x, pos_msg.y, pos_msg.z);
        geometry_msgs::msg::Quaternion quat_msg = corrent_pose_stamped_.pose.orientation;
        Eigen::Quaterniond previous_quat = Eigen::Quaterniond(quat_msg.w, quat_msg.x, quat_msg.y, quat_msg.z);
        Eigen::Matrix3d rot_mat = previous_quat.toRotationMatrix();
        Eigen::Vector3d acc = Eigen::Vector3d(imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z);

        // pos
        pos = pos + dt_imu * vec_imu_ + 0.5 * dt_imu * dt_imu * (rot_mat * acc - gravity_); 
        // vel
        vec_imu_ = vec_imu_ + dt_imu * (rot_mat * acc - gravity_);
        // quat 
        Eigen::Quaterniond quat_wdt =  Eigen::Quaterniond(Eigen::AngleAxisd(imu_msg.angular_velocity.x * dt_imu, Eigen::Vector3d::UnitX()) 
                                        * Eigen::AngleAxisd(imu_msg.angular_velocity.y * dt_imu, Eigen::Vector3d::UnitY())    
                                        * Eigen::AngleAxisd(imu_msg.angular_velocity.z * dt_imu, Eigen::Vector3d::UnitZ()));  
        Eigen::Quaterniond predicted_quat = quat_wdt * previous_quat;

        corrent_pose_stamped_.header.stamp = imu_msg.header.stamp;
        //corrent_pose_stamped_.pose.position.x = pos.x();
        //corrent_pose_stamped_.pose.position.y = pos.y();
        //corrent_pose_stamped_.pose.position.z = pos.z();
        corrent_pose_stamped_.pose.orientation.x = predicted_quat.x();
        corrent_pose_stamped_.pose.orientation.y = predicted_quat.y();
        corrent_pose_stamped_.pose.orientation.z = predicted_quat.z();
        corrent_pose_stamped_.pose.orientation.w = predicted_quat.w();
        pose_pub_->publish(corrent_pose_stamped_);

        return;
    }


}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(graphslam::ScanMatcherComponent)
