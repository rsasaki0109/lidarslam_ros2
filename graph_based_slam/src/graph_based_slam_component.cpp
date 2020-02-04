#include "graph_based_slam/graph_based_slam_component.h"
#include <chrono>

using namespace std::chrono_literals;

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
        declare_parameter("threshold_loop_clousure", 1.0);
        get_parameter("threshold_loop_clousure", threshold_loop_clousure_);
        declare_parameter("distance_loop_clousure", 4.5);
        get_parameter("distance_loop_clousure", distance_loop_clousure_);

        std::cout << "voxel_leaf_size[m]:" << voxel_leaf_size << std::endl;
        std::cout << "ndt_resolution[m]:" << ndt_resolution << std::endl;
        std::cout << "loop_detection_period[Hz]:" << loop_detection_period_ << std::endl;
        std::cout << "threshold_loop_clousure:" << threshold_loop_clousure_ << std::endl;
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
            map_array_msg_ = *msg_ptr;
        };

        map_array_sub_ = 
            create_subscription<graphslam_ros2_msgs::msg::MapArray>(
                "map_array", rclcpp::SystemDefaultsQoS(), map_array_callback);  

        std::chrono::milliseconds period(loop_detection_period_);
        loop_detect_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period), 
            std::bind(&GraphBasedSlamComponent::searchLoop, this)
            );

        modified_map_array_pub_ = create_publisher<graphslam_ros2_msgs::msg::MapArray>(
            "modified_map_array", rclcpp::SystemDefaultsQoS());     
        
    }

    //TODO:searching for multiple recenet submaps 
    void GraphBasedSlamComponent::searchLoop()
    {
        graphslam_ros2_msgs::msg::MapArray map_array_msg = map_array_msg_;

        graphslam_ros2_msgs::msg::SubMap latest_submap;
        latest_submap = map_array_msg_.submaps.back();
        pcl::PointCloud<pcl::PointXYZI>::Ptr latest_submap_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(latest_submap.cloud, *latest_submap_cloud_ptr);
        ndt_.setInputTarget(latest_submap_cloud_ptr);
        double latest_moving_distance = latest_submap.distance;
        int i =0;
        for(auto submap : map_array_msg_.submaps){

            if(latest_moving_distance - submap.distance > distance_loop_clousure_){

                pcl::PointCloud<pcl::PointXYZI>::Ptr submap_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
                pcl::fromROSMsg(submap.cloud, *submap_cloud_ptr);
                ndt_.setInputSource(submap_cloud_ptr);

                pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
                ndt_.align(*output_cloud_ptr);

                double fitness_score = ndt_.getFitnessScore();
                if(fitness_score < threshold_loop_clousure_){
                    doPoseAdjustment(i, map_array_msg);
                    return;
                }
                i++;
            }
        }
        


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

            g2o::VertexSE3* vertex_sim3 = new g2o::VertexSE3();
            vertex_sim3->setId(i);
            vertex_sim3->setEstimate(pose);
            if (i == submaps_size - 1) vertex_sim3->setFixed(true);
            optimizer.addVertex(vertex_sim3);

            if(i=id_loop_point) first_visited_point = pose;
            if(i>0){
                Eigen::Isometry3d relative_pose = pose.inverse() * previous_pose;
    
                g2o::EdgeSE3* edge_sim3 = new g2o::EdgeSE3();
                edge_sim3->setMeasurement(relative_pose);
                edge_sim3->setInformation(info_mat);
                edge_sim3->vertices()[0] = optimizer.vertex(i-1);
                edge_sim3->vertices()[1] = optimizer.vertex(i);
                optimizer.addEdge(edge_sim3);
            }
            
            previous_pose = pose;
        }

        // loop edge
        Eigen::Isometry3d revisit_point = previous_pose;
        Eigen::Isometry3d relative_pose = first_visited_point.inverse() * revisit_point;
        g2o::EdgeSE3* edge_sim3 = new g2o::EdgeSE3();
        edge_sim3->setMeasurement(relative_pose);
        edge_sim3->setInformation(info_mat);
        edge_sim3->vertices()[0] = optimizer.vertex(id_loop_point);
        edge_sim3->vertices()[1] = optimizer.vertex(submaps_size-1);
        optimizer.addEdge(edge_sim3);

        optimizer.initializeOptimization();
        optimizer.optimize(10);

        graphslam_ros2_msgs::msg::MapArray modified_map_array_msg;
        modified_map_array_msg.header = map_array_msg.header;
        for(int i = 0;  i  < submaps_size ; i++ ){
            g2o::VertexSE3* vertex_sim3 = static_cast<g2o::VertexSE3*>(optimizer.vertex(i));
            auto se3 = vertex_sim3->estimate();
            auto translation = se3.translation();
            tf2::Matrix3x3 rotation_matrix;
            rotation_matrix.setValue(
                       static_cast<double>(translation(0, 0)), static_cast<double>(translation(0, 1)),
                       static_cast<double>(translation(0, 2)), static_cast<double>(translation(1, 0)),
                       static_cast<double>(translation(1, 1)), static_cast<double>(translation(1, 2)),
                       static_cast<double>(translation(2, 0)), static_cast<double>(translation(2, 1)),
                       static_cast<double>(translation(2, 2)));

            geometry_msgs::msg::Point pos;
            pos.x = translation(0, 3);
            pos.x = translation(1, 3);
            pos.x = translation(2, 3);
            tf2::Quaternion quat_tf2;
            rotation_matrix.getRotation(quat_tf2);
            geometry_msgs::msg::Quaternion quat;
            quat.x = quat_tf2.x();
            quat.y = quat_tf2.y();
            quat.z = quat_tf2.z();
            quat.w = quat_tf2.w();
            
            graphslam_ros2_msgs::msg::SubMap submap;
            submap.header = map_array_msg.submaps[i].header;
            submap.pose.position = pos;
            submap.pose.orientation = quat;
            submap.cloud = map_array_msg.submaps[i].cloud;
            
            modified_map_array_msg.submaps.push_back(submap);
        }
        modified_map_array_pub_->publish(modified_map_array_msg);
    }
 
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(graphslam::GraphBasedSlamComponent)