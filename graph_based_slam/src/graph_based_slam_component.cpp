#include "graph_based_slam/graph_based_slam_component.h"

#include <chrono>
#include <sstream>

using namespace std::chrono_literals;

namespace graphslam
{

GraphBasedSlamComponent::GraphBasedSlamComponent(const rclcpp::NodeOptions & options)
: Node("graph_based_slam", options),
  clock_(RCL_ROS_TIME),
  tfbuffer_(std::make_shared<rclcpp::Clock>(clock_)),
  listener_(tfbuffer_),
  broadcaster_(this),
  use_save_map_in_loop_(true),
  initial_map_array_received_(false),
  is_map_array_updated_(false),
  previous_submaps_num_(0),
  debug_flag_(false)
{
  RCLCPP_INFO(get_logger(), "initialization start");
  std::string registration_method;
  double voxel_leaf_size;
  double ndt_resolution;
  int ndt_num_threads;

  registration_method = declare_parameter("registration_method", "NDT");
  voxel_leaf_size = declare_parameter("voxel_leaf_size", 0.2);
  ndt_resolution = declare_parameter("ndt_resolution", 5.0);
  ndt_num_threads = declare_parameter("ndt_num_threads", 0);
  loop_detection_period_ = declare_parameter("loop_detection_period", 1000);
  threshold_loop_closure_score_ = declare_parameter("threshold_loop_closure_score", 1.0);
  distance_loop_closure_ = declare_parameter("distance_loop_closure", 20.0);
  range_of_searching_loop_closure_ = declare_parameter("range_of_searching_loop_closure", 20.0);
  search_submap_num_ = declare_parameter("search_submap_num", 3);
  num_adjacent_pose_cnstraints_ = declare_parameter("num_adjacent_pose_cnstraints", 5);
  use_save_map_in_loop_ = declare_parameter("use_save_map_in_loop", true);
  use_save_map_in_loop_ = get_parameter("use_save_map_in_loop", use_save_map_in_loop_);
  debug_flag_ = declare_parameter("debug_flag", false);
  map_frame_id_ = declare_parameter("map_frame_id", "map");
  odom_frame_id_ = declare_parameter("odom_frame_id", "odom");
  map_array_topic_ = declare_parameter("map_array_topic", "map_array");

  std::stringstream ss;
  ss << "SETTINGS" << std::endl;
  ss << "registration_method: " << registration_method << std::endl;
  ss << "voxel_leaf_size[m]: " << voxel_leaf_size << std::endl;
  ss << "ndt_resolution[m]: " << ndt_resolution << std::endl;
  ss << "ndt_num_threads: " << ndt_num_threads << std::endl;
  ss << "loop_detection_period[Hz]: " << loop_detection_period_ << std::endl;
  ss << "threshold_loop_closure_score: " << threshold_loop_closure_score_ << std::endl;
  ss << "distance_loop_closure[m]: " << distance_loop_closure_ << std::endl;
  ss << "range_of_searching_loop_closure[m]: " << range_of_searching_loop_closure_ << std::endl;
  ss << "search_submap_num: " << search_submap_num_ << std::endl;
  ss << "num_adjacent_pose_cnstraints: " << num_adjacent_pose_cnstraints_ << std::endl;
  ss << "use_save_map_in_loop: " << use_save_map_in_loop_ << std::endl;
  ss << "debug_flag: " << debug_flag_;

  RCLCPP_INFO(get_logger(), ss.str());

  voxelgrid_.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);

  if (registration_method == "NDT") {
    pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr
      ndt(new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
    ndt->setMaximumIterations(100);
    ndt->setResolution(ndt_resolution);
    ndt->setTransformationEpsilon(0.01);
    // ndt->setTransformationEpsilon(1e-6);
    ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    if (ndt_num_threads > 0) {ndt->setNumThreads(ndt_num_threads);}
    registration_ = ndt;
  } else {
    pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>::Ptr
      gicp(new pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>());
    gicp->setMaxCorrespondenceDistance(30);
    gicp->setMaximumIterations(100);
    //gicp->setCorrespondenceRandomness(20);
    gicp->setTransformationEpsilon(1e-8);
    gicp->setEuclideanFitnessEpsilon(1e-6);
    gicp->setRANSACIterations(0);
    registration_ = gicp;
  }

  // publishers
  modified_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "modified_map",
    rclcpp::QoS(10));

  modified_map_array_pub_ = create_publisher<lidarslam_msgs::msg::MapArray>(
    "modified_map_array", rclcpp::QoS(10));

  modified_path_pub_ = create_publisher<nav_msgs::msg::Path>(
    "modified_path",
    rclcpp::QoS(10));

  // subscribers
  map_array_sub_ = create_subscription<lidarslam_msgs::msg::MapArray>(
    map_array_topic_,
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
    std::bind(&GraphBasedSlamComponent::mapArrayCallback, this, std::placeholders::_1)
  );

  // services
  map_save_srv_ = create_service<std_srvs::srv::Empty>("map_save",
    std::bind(&GraphBasedSlamComponent::mapSaveCallback, this, std::placeholders::_1, std::placeholders::_2)
  );

  // timers
  std::chrono::milliseconds period(loop_detection_period_);
  loop_detect_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&GraphBasedSlamComponent::searchLoop, this)
  );

  odom2map_timer_ = create_wall_timer(
    50ms,
    std::bind(&GraphBasedSlamComponent::broadcastOdom2Map, this)
  );

  RCLCPP_INFO(get_logger(), "Listening to (%s)...", map_array_topic_.c_str());
}

void GraphBasedSlamComponent::searchLoop()
{
  if (initial_map_array_received_ == false) {return;}
  if (is_map_array_updated_ == false) {return;}
  if (map_array_msg_.cloud_coordinate != map_array_msg_.LOCAL) {
    RCLCPP_WARN(get_logger(), "cloud_coordinate should be local, but it's not local.");
  }
  is_map_array_updated_ = false;

  lidarslam_msgs::msg::MapArray map_array_msg = map_array_msg_;
  int num_submaps = map_array_msg.submaps.size();

  RCLCPP_INFO(get_logger(), "searching Loop, num_submaps: %i", num_submaps);

  double min_fitness_score = std::numeric_limits<double>::max();
  double distance_min_fitness_score = 0;
  bool is_candidate = false;

  lidarslam_msgs::msg::SubMap latest_submap;
  latest_submap = map_array_msg.submaps[num_submaps - 1];
  Eigen::Affine3d latest_submap_affine;
  tf2::fromMsg(latest_submap.pose, latest_submap_affine);
  pcl::PointCloud<pcl::PointXYZI>::Ptr latest_submap_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(latest_submap.cloud, *latest_submap_cloud_ptr);
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_latest_submap_cloud_ptr(
    new pcl::PointCloud<pcl::PointXYZI>);
  Eigen::Affine3d latest_affine;
  tf2::fromMsg(latest_submap.pose, latest_affine);
  pcl::transformPointCloud(
    *latest_submap_cloud_ptr, *transformed_latest_submap_cloud_ptr,
    latest_affine.matrix().cast<float>());
  registration_->setInputSource(transformed_latest_submap_cloud_ptr);
  double latest_moving_distance = latest_submap.distance;
  Eigen::Vector3d latest_submap_pos{
    latest_submap.pose.position.x,
    latest_submap.pose.position.y,
    latest_submap.pose.position.z};

  int id_min = 0;
  double min_dist = std::numeric_limits<double>::max();
  lidarslam_msgs::msg::SubMap min_submap;
  for (int i = 0; i < num_submaps - 1; i++) {
    auto submap = map_array_msg.submaps[i];
    Eigen::Vector3d submap_pos{submap.pose.position.x, submap.pose.position.y,
      submap.pose.position.z};
    double dist = (latest_submap_pos - submap_pos).norm();

    RCLCPP_INFO(get_logger(), ss.str());

    if (latest_moving_distance - submap.distance > distance_loop_closure_ &&
      dist < range_of_searching_loop_closure_)
    {
      is_candidate = true;
      if (dist < min_dist) {
        id_min = i;
        min_dist = dist;
        min_submap = submap;
      }
    }
  }
  if (is_candidate) {
    RCLCPP_INFO(get_logger(), "candidate found");
    pcl::PointCloud<pcl::PointXYZI>::Ptr submap_clouds_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    for (int j = 0; j <= 2 * search_submap_num_; ++j) {
      if (id_min + j - search_submap_num_ < 0) {continue;}
      auto near_submap = map_array_msg.submaps[id_min + j - search_submap_num_];
      pcl::PointCloud<pcl::PointXYZI>::Ptr submap_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::fromROSMsg(near_submap.cloud, *submap_cloud_ptr);
      pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_submap_cloud_ptr(
        new pcl::PointCloud<pcl::PointXYZI>);
      Eigen::Affine3d affine;
      tf2::fromMsg(near_submap.pose, affine);
      pcl::transformPointCloud(
        *submap_cloud_ptr, *transformed_submap_cloud_ptr,
        affine.matrix().cast<float>());
      *submap_clouds_ptr += *transformed_submap_cloud_ptr;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_clouds_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    voxelgrid_.setInputCloud(submap_clouds_ptr);
    voxelgrid_.filter(*filtered_clouds_ptr);
    registration_->setInputTarget(filtered_clouds_ptr);

    pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    registration_->align(*output_cloud_ptr);
    double fitness_score = registration_->getFitnessScore();
    RCLCPP_INFO(get_logger(), "fitness: %f, threshold: %f ", fitness_score, threshold_loop_closure_score_);
    if (fitness_score < threshold_loop_closure_score_) {

      Eigen::Affine3d init_affine;
      tf2::fromMsg(latest_submap.pose, init_affine);
      Eigen::Affine3d submap_affine;
      tf2::fromMsg(min_submap.pose, submap_affine);

      LoopEdge loop_edge;
      loop_edge.pair_id = std::pair<int, int>(id_min, num_submaps - 1);
      Eigen::Isometry3d from = Eigen::Isometry3d(submap_affine.matrix());
      Eigen::Isometry3d to = Eigen::Isometry3d(
        registration_->getFinalTransformation().cast<double>() * init_affine.matrix());

      loop_edge.relative_pose = Eigen::Isometry3d(from.inverse() * to);
      loop_edges_.push_back(loop_edge);

      std::stringstream ss;
      ss << "PoseAdjustment" << std::endl;
      ss << "distance: " << min_submap.distance << "fitness_score: " << fitness_score << std::endl;
      ss << "id_loop_point 1: " << id_min << std::endl;
      ss << "id_loop_point 2: " << num_submaps - 1 << std::endl;
      ss << "final transformation: " << registration_->getFinalTransformation() << std::endl;

      RCLCPP_INFO(get_logger(), ss.str());
      doPoseAdjustment(use_save_map_in_loop_);

      return;
    }

    RCLCPP_INFO(get_logger(), "min_submap_distance: %f", min_submap.distance);
    RCLCPP_INFO(get_logger(), "fitness_score: %f", fitness_score);
  }
}

void GraphBasedSlamComponent::doPoseAdjustment(bool save_map)
{
  g2o::SparseOptimizer optimizer;
  optimizer.setVerbose(false);
  std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linear_solver =
    g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
  g2o::OptimizationAlgorithmLevenberg * solver = new g2o::OptimizationAlgorithmLevenberg(
    g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linear_solver)));

  optimizer.setAlgorithm(solver);

  int submaps_size = map_array_msg_.submaps.size();
  Eigen::Matrix<double, 6, 6> info_mat = Eigen::Matrix<double, 6, 6>::Identity();
  for (int i = 0; i < submaps_size; i++) {
    Eigen::Affine3d affine;
    Eigen::fromMsg(map_array_msg_.submaps[i].pose, affine);
    Eigen::Isometry3d pose(affine.matrix());

    g2o::VertexSE3 * vertex_se3 = new g2o::VertexSE3();
    vertex_se3->setId(i);
    vertex_se3->setEstimate(pose);
    if (i == submaps_size - 1) {vertex_se3->setFixed(true);}
    optimizer.addVertex(vertex_se3);

    if (i > num_adjacent_pose_cnstraints_) {
      for (int j = 0; j < num_adjacent_pose_cnstraints_; j++) {
        Eigen::Affine3d pre_affine;
        Eigen::fromMsg(
          map_array_msg_.submaps[i - num_adjacent_pose_cnstraints_ + j].pose,
          pre_affine);
        Eigen::Isometry3d pre_pose(pre_affine.matrix());
        Eigen::Isometry3d relative_pose = pre_pose.inverse() * pose;
        g2o::EdgeSE3 * edge_se3 = new g2o::EdgeSE3();
        edge_se3->setMeasurement(relative_pose);
        edge_se3->setInformation(info_mat);
        edge_se3->vertices()[0] = optimizer.vertex(i - num_adjacent_pose_cnstraints_ + j);
        edge_se3->vertices()[1] = optimizer.vertex(i);
        optimizer.addEdge(edge_se3);
      }
    }

  }
  /* loop edge */
  for (auto loop_edge : loop_edges_) {
    g2o::EdgeSE3 * edge_se3 = new g2o::EdgeSE3();
    edge_se3->setMeasurement(loop_edge.relative_pose);
    edge_se3->setInformation(info_mat);
    edge_se3->vertices()[0] = optimizer.vertex(loop_edge.pair_id.first);
    edge_se3->vertices()[1] = optimizer.vertex(loop_edge.pair_id.second);
    optimizer.addEdge(edge_se3);
  }

  optimizer.initializeOptimization();
  optimizer.optimize(10);

  /* modified_map publish */
  RCLCPP_INFO(get_logger(), "modified_map publish");
  lidarslam_msgs::msg::MapArray modified_map_array_msg;
  modified_map_array_msg.header = map_array_msg_.header;
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  for (int i = 0; i < submaps_size; i++) {
    g2o::VertexSE3 * vertex_se3 = static_cast<g2o::VertexSE3 *>(optimizer.vertex(i));
    Eigen::Affine3d se3 = vertex_se3->estimate();
    geometry_msgs::msg::Pose pose = tf2::toMsg(se3);

    /* map */
    Eigen::Affine3d previous_affine;
    tf2::fromMsg(map_array_msg_.submaps[i].pose, previous_affine);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud_ptr(
      new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(map_array_msg_.submaps[i].cloud, *cloud_ptr);

    pcl::transformPointCloud(*cloud_ptr, *transformed_cloud_ptr, se3.matrix().cast<float>());
    sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg_ptr(new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(*transformed_cloud_ptr, *cloud_msg_ptr);
    *map_ptr += *transformed_cloud_ptr;

    /* submap */
    lidarslam_msgs::msg::SubMap submap;
    submap.header = map_array_msg_.submaps[i].header;
    submap.pose = pose;
    submap.cloud = *cloud_msg_ptr;
    modified_map_array_msg.submaps.push_back(submap);

    /* path */
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = submap.header;
    pose_stamped.pose = submap.pose;
    path.poses.push_back(pose_stamped);

  }
  modified_map_array_pub_->publish(modified_map_array_msg);
  modified_path_pub_->publish(path);

  sensor_msgs::msg::PointCloud2::SharedPtr map_msg_ptr(new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(*map_ptr, *map_msg_ptr);
  map_msg_ptr->header.frame_id = "map";
  modified_map_pub_->publish(*map_msg_ptr);

  if (save_map) {
    optimizer.save("pose_graph.g2o");
    pcl::io::savePCDFileASCII("map.pcd", *map_ptr);
  }
}

void GraphBasedSlamComponent::broadcastOdom2Map()
{
  if (!initial_map_array_received_)
  {
    // no pose to broadcast yet
    return;
  }
  auto first_kf_pose = map_array_msg_.submaps[0].pose;
  geometry_msgs::msg::TransformStamped transform_msg;

  transform_msg.header.stamp = get_clock()->now();
  transform_msg.header.frame_id = map_frame_id_;
  transform_msg.child_frame_id = odom_frame_id_;
  transform_msg.transform.translation.x = first_kf_pose.position.x;
  transform_msg.transform.translation.y = first_kf_pose.position.y;
  transform_msg.transform.translation.z = first_kf_pose.position.y;
  transform_msg.transform.rotation.x = first_kf_pose.orientation.x;
  transform_msg.transform.rotation.y = first_kf_pose.orientation.y;
  transform_msg.transform.rotation.z = first_kf_pose.orientation.z;
  transform_msg.transform.rotation.w = first_kf_pose.orientation.w;

  broadcaster_.sendTransform(transform_msg);
}

void GraphBasedSlamComponent::mapArrayCallback(const lidarslam_msgs::msg::MapArray::SharedPtr msg_ptr)
{
    map_array_msg_ = *msg_ptr;
    initial_map_array_received_ = true;
    is_map_array_updated_ = true;
}

void GraphBasedSlamComponent::mapSaveCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
  const std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  RCLCPP_INFO(get_logger(), "Received an request to save the map");
  if (initial_map_array_received_ == false) {
    RCLCPP_INFO(get_logger(), "initial map is not received");
    return;
  }
  doPoseAdjustment(true);
}

} // namespace graphslam

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(graphslam::GraphBasedSlamComponent)

