#include "graph_based_slam/graph_based_slam_component.h"
#include <chrono>

using namespace std::chrono_literals;

std::mutex mtx;

namespace graphslam
{
    GraphBasedSlamComponent::GraphBasedSlamComponent(const rclcpp::NodeOptions & options)
    : Node("graph_based_slam", options),
        clock_(RCL_ROS_TIME), 
        tfbuffer_(std::make_shared<rclcpp::Clock>(clock_)),
        listener_(tfbuffer_),
        broadcaster_(this)
    {
        double voxel_leaf_size;
        double ndt_resolution;

        declare_parameter("voxel_leaf_size", 0.2);
        get_parameter("voxel_leaf_size", voxel_leaf_size);
        declare_parameter("ndt_resolution", 5.0);
        get_parameter("ndt_resolution", ndt_resolution);
        declare_parameter("loop_detection_period", 1000);
        get_parameter("loop_detection_period", loop_detection_period_);
        declare_parameter("threshold_loop_clousure_score", 1.0);
        get_parameter("threshold_loop_clousure_score", threshold_loop_clousure_score_);
        declare_parameter("distance_loop_clousure", 4.5);
        get_parameter("distance_loop_clousure", distance_loop_clousure_);

        std::cout << "voxel_leaf_size[m]:" << voxel_leaf_size << std::endl;
        std::cout << "ndt_resolution[m]:" << ndt_resolution << std::endl;
        std::cout << "loop_detection_period[Hz]:" << loop_detection_period_ << std::endl;
        std::cout << "threshold_loop_clousure_score:" << threshold_loop_clousure_score_ << std::endl;
        std::cout << "distance_loop_clousure:" << distance_loop_clousure_ << std::endl;
        std::cout << "------------------" << std::endl;

        voxelgrid_.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);

        ndt_.setResolution(ndt_resolution);

        initializePubSub();
        RCLCPP_INFO(get_logger(), "initialization end");
    }   

    void GraphBasedSlamComponent::initializePubSub(){
        RCLCPP_INFO(get_logger(), "initialize Publishers and Subscribers");

        auto map_array_callback =
        [this](const typename graphslam_ros2_msgs::msg::MapArray::SharedPtr msg_ptr) -> void
        {
            //TODO:mutex
            std::lock_guard<std::mutex> lock(mtx);
            map_array_msg_ = *msg_ptr;
            initial_map_array_received_ = true;
            is_map_array_updated_ = true;
        };

        map_array_sub_ = 
            create_subscription<graphslam_ros2_msgs::msg::MapArray>(
                "map_array", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(), map_array_callback);  

        std::chrono::milliseconds period(loop_detection_period_);
        loop_detect_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period), 
            std::bind(&GraphBasedSlamComponent::searchLoop, this)
            );
            

        modified_map_array_pub_ = create_publisher<graphslam_ros2_msgs::msg::MapArray>(
            "modified_map_array", rclcpp::SystemDefaultsQoS()); 

        modified_path_pub_ = create_publisher<nav_msgs::msg::Path>("modified_path", rclcpp::SystemDefaultsQoS());    
        
    }

    //TODO:searching for multiple recenet submaps 
    void GraphBasedSlamComponent::searchLoop()
    {

        if(initial_map_array_received_ == false) return;
        if(is_map_array_updated_ == false) return;
        is_map_array_updated_ = false;

        std::lock_guard<std::mutex> lock(mtx);
        std::cout << "----------------------------" << std::endl;
        std::cout << "do searchLoop" << std::endl;

        graphslam_ros2_msgs::msg::MapArray map_array_msg = map_array_msg_;
        int num_submaps = map_array_msg_.submaps.size();
        std::cout << "num_submaps:" << num_submaps << std::endl;

        graphslam_ros2_msgs::msg::SubMap latest_submap;
        latest_submap = map_array_msg_.submaps.back();
        pcl::PointCloud<pcl::PointXYZI>::Ptr latest_submap_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(latest_submap.cloud, *latest_submap_cloud_ptr);
        ndt_.setInputTarget(latest_submap_cloud_ptr);
        double latest_moving_distance = latest_submap.distance;
        Eigen::Vector3d latest_submap_pos{latest_submap.pose.position.x, latest_submap.pose.position.y, latest_submap.pose.position.z};
        int i =0;
        double min_fitness_score = 1000;
        double distance_min_fitness_score = 0;
        std::cout << "latest_moving_distance:" << latest_moving_distance << std::endl;
        geometry_msgs::msg::PoseStamped pose_stamped_minsocore;
        bool is_candidate = false;
        for(auto submap : map_array_msg_.submaps){
            
            Eigen::Vector3d submap_pos{submap.pose.position.x, submap.pose.position.y, submap.pose.position.z};
            if(latest_moving_distance - submap.distance > distance_loop_clousure_ && (latest_submap_pos - submap_pos).norm() < distance_loop_clousure_){
                is_candidate = true;
                //std::cout << "-" << std::endl;
                //std::cout << "submap.distance:" << submap.distance <<std::endl;
                //std::cout << "delta_pos:" << (latest_submap_pos - submap_pos).norm() <<std::endl;

                pcl::PointCloud<pcl::PointXYZI>::Ptr submap_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
                pcl::fromROSMsg(submap.cloud, *submap_cloud_ptr);
                ndt_.setInputSource(submap_cloud_ptr);

                pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
                ndt_.align(*output_cloud_ptr);

                double fitness_score = ndt_.getFitnessScore();
                std::cout << "fitness_score:" << fitness_score << std::endl;
                if(fitness_score < threshold_loop_clousure_score_){
                    std::cout << "do PoseAdjustment" << std::endl;
                    doPoseAdjustment(i, map_array_msg);
                    return;
                }

                if(fitness_score < min_fitness_score){
                    distance_min_fitness_score = submap.distance;
                    min_fitness_score = fitness_score;
                    pose_stamped_minsocore.header = submap.header;
                    pose_stamped_minsocore.pose = submap.pose;
                }

                i++;
            }
        }
        if(is_candidate){
            std::cout << "-"  << std::endl;
            std::cout << "distance_min_fitness_score:" << distance_min_fitness_score << std::endl;
            std::cout << "min_fitness_score:" << min_fitness_score << std::endl;
            std::cout << "pose_minsocore"  << std::endl;
            std::cout << "x:" << pose_stamped_minsocore.pose.position.x << "," <<
                         "y:" << pose_stamped_minsocore.pose.position.y << "," <<
                         "z:" << pose_stamped_minsocore.pose.position.z << std::endl;
        }   

        std::cout << "searchLoop end" << std::endl;

    }

    void GraphBasedSlamComponent::doPoseAdjustment(int id_loop_point, graphslam_ros2_msgs::msg::MapArray map_array_msg){

        g2o::SparseOptimizer optimizer;
        optimizer.setVerbose(false);
        std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linear_solver =
            g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
        g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
		    g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linear_solver)));

        optimizer.setAlgorithm(solver);

        int submaps_size = map_array_msg_.submaps.size();
        Eigen::Isometry3d first_visited_point;
        Eigen::Isometry3d previous_pose;
        Eigen::Matrix<double, 6, 6> info_mat = Eigen::Matrix<double, 6, 6>::Identity();
        for(int i = 0;  i  < submaps_size ; i++ ){
            geometry_msgs::msg::Point pos = map_array_msg_.submaps[i].pose.position;
            geometry_msgs::msg::Quaternion quat = map_array_msg_.submaps[i].pose.orientation;
            Eigen::Vector3d translation(pos.x, pos.y, pos.z);
            Eigen::Quaterniond rotation(quat.w, quat.x, quat.y, quat.z);
            Eigen::Isometry3d pose(Eigen::Isometry3d::Identity());
		    pose.pretranslate(translation);
            pose.prerotate(rotation);

            g2o::VertexSE3* vertex_se3 = new g2o::VertexSE3();
            vertex_se3->setId(i);
            vertex_se3->setEstimate(pose);
            if (i == submaps_size - 1) vertex_se3->setFixed(true);
            optimizer.addVertex(vertex_se3);

            if(i == id_loop_point) first_visited_point = pose;
            if(i>0){
                Eigen::Isometry3d relative_pose = pose.inverse() * previous_pose;
    
                g2o::EdgeSE3* edge_se3 = new g2o::EdgeSE3();
                edge_se3->setMeasurement(relative_pose);
                edge_se3->setInformation(info_mat);
                edge_se3->vertices()[0] = optimizer.vertex(i-1);
                edge_se3->vertices()[1] = optimizer.vertex(i);
                optimizer.addEdge(edge_se3);
            }
            
            previous_pose = pose;
        }
        // loop edge
        Eigen::Isometry3d revisit_point = previous_pose;
        Eigen::Isometry3d relative_pose = first_visited_point.inverse() * revisit_point;
        g2o::EdgeSE3* edge_se3 = new g2o::EdgeSE3();
        edge_se3->setMeasurement(relative_pose);
        edge_se3->setInformation(info_mat);
        edge_se3->vertices()[0] = optimizer.vertex(id_loop_point);
        edge_se3->vertices()[1] = optimizer.vertex(submaps_size-1);
        optimizer.addEdge(edge_se3);

        optimizer.initializeOptimization();
        optimizer.optimize(10);

        graphslam_ros2_msgs::msg::MapArray modified_map_array_msg;
        modified_map_array_msg.header = map_array_msg.header;
        nav_msgs::msg::Path path;
        path.header.frame_id = "map";
        for(int i = 0;  i  < submaps_size ; i++ ){
            g2o::VertexSE3* vertex_se3 = static_cast<g2o::VertexSE3*>(optimizer.vertex(i));
            auto se3 = vertex_se3->estimate();
            Eigen::Vector3d translation = se3.translation();
            Eigen::Matrix3d rotation = se3.rotation();
            
            geometry_msgs::msg::Point pos;
            pos.x = translation(0);
            pos.x = translation(1);
            pos.x = translation(2);

            Eigen::Quaterniond q_eig(rotation);
            geometry_msgs::msg::Quaternion quat = tf2::toMsg(q_eig); 

            graphslam_ros2_msgs::msg::SubMap submap;
            submap.header = map_array_msg.submaps[i].header;
            submap.pose.position = pos;
            submap.pose.orientation = quat;
            submap.cloud = map_array_msg.submaps[i].cloud;
            
            modified_map_array_msg.submaps.push_back(submap);

            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header = submap.header;
            pose_stamped.pose = submap.pose;
            path.poses.push_back(pose_stamped);
        }
        std::cout << "publish" << std::endl;
        modified_map_array_pub_->publish(modified_map_array_msg);
        modified_path_pub_->publish(path);
    }
 
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(graphslam::GraphBasedSlamComponent)