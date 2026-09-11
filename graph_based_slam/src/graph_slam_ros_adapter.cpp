// Copyright 2026 Sasaki
// All rights reserved.
//
// Software License Agreement (BSD 2-Clause Simplified License)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "graph_based_slam_component_impl.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

#include <pcl/common/common.h>  // NOLINT(build/include_order)
#include <pcl/filters/voxel_grid.h>  // NOLINT(build/include_order)
#include <pcl_conversions/pcl_conversions.h>  // NOLINT(build/include_order)
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include "filesystem_io_ports.hpp"
#include "graph_slam_composition.hpp"
#include "graph_based_slam/adjacent_edge_auto_scale.hpp"
#include "graph_based_slam/backend_core.hpp"
#include "graph_based_slam/bev_mutual_visibility.hpp"
#include "graph_based_slam/candidate_aggregator.hpp"
#include "graph_based_slam/degeneracy_diagnostics_csv.hpp"
#include "graph_based_slam/dynamic_object_filter.hpp"
#include "graph_based_slam/gnss_alignment.hpp"
#include "graph_based_slam/gnss_geometry.hpp"
#include "graph_based_slam/gnss_weighting.hpp"
#include "graph_based_slam/graph_slam_application.hpp"
#include "graph_based_slam/loop_verifier.hpp"
#include "graph_based_slam/map_saver.hpp"
#include "graph_based_slam/planar_map_filter.hpp"
#include "graph_based_slam/pointcloud_conversion.hpp"
#include "graph_based_slam/pose_graph_optimization.hpp"
#include "graph_based_slam/submap_creation.hpp"
#include "g2o/core/robust_kernel_impl.h"
#define GRAPH_BASED_SLAM_WITH_G2O 1
#include "graph_based_slam/loop_edge_robustifier.hpp"

using namespace std::chrono_literals;

namespace graphslam
{
namespace
{
class RosClockPort final : public ports::ClockPort
{
public:
  explicit RosClockPort(rclcpp::Clock::SharedPtr clock)
  : clock_(std::move(clock)) {}

  double nowSeconds() const override {return clock_->now().seconds();}

private:
  rclcpp::Clock::SharedPtr clock_;
};

class RosDiagnosticsPort final : public ports::DiagnosticsPort
{
public:
  explicit RosDiagnosticsPort(rclcpp::Logger logger)
  : logger_(std::move(logger)) {}

  void emit(const ports::DiagnosticEvent & event) override
  {
    switch (event.level) {
      case ports::DiagnosticLevel::DEBUG:
        RCLCPP_DEBUG(logger_, "[%s] %s", event.code.c_str(), event.message.c_str());
        break;
      case ports::DiagnosticLevel::INFO:
        RCLCPP_INFO(logger_, "[%s] %s", event.code.c_str(), event.message.c_str());
        break;
      case ports::DiagnosticLevel::WARNING:
        RCLCPP_WARN(logger_, "[%s] %s", event.code.c_str(), event.message.c_str());
        break;
      case ports::DiagnosticLevel::ERROR:
        RCLCPP_ERROR(logger_, "[%s] %s", event.code.c_str(), event.message.c_str());
        break;
    }
  }

private:
  rclcpp::Logger logger_;
};

ports::ExternalIoPorts makeExternalIoPorts(
  rclcpp::Node & node, const GraphSlamConfig & config)
{
  ports::ExternalIoPorts result;
  result.clock = std::make_shared<RosClockPort>(node.get_clock());
  if (config.use_pcd_cache_) {
    result.submap_storage =
      std::make_shared<adapters::PcdSubmapStorage>(config.pcd_cache_dir_);
  } else {
    result.submap_storage = std::make_shared<ports::InMemorySubmapStorage>();
  }
  result.diagnostics = std::make_shared<RosDiagnosticsPort>(node.get_logger());
  result.map_output = std::make_shared<adapters::FilesystemMapOutput>();
  result.validate();
  return result;
}

GraphSlamConfig loadValidatedGraphSlamConfig(rclcpp::Node & node)
{
  GraphSlamConfig config = loadGraphSlamConfig(node);
  const ConfigNormalization normalization = normalizeGraphSlamConfig(config);
  for (const auto & warning : normalization.warnings) {
    RCLCPP_WARN(node.get_logger(), "graph SLAM config: %s", warning.c_str());
  }

  const auto errors = validateGraphSlamConfig(config);
  if (!errors.empty()) {
    for (const auto & error : errors) {
      RCLCPP_ERROR(node.get_logger(), "invalid graph SLAM configuration: %s", error.c_str());
    }
    throw std::invalid_argument(errors.front());
  }
  return config;
}
}  // namespace

struct GraphBasedSlamComponent::Impl::BackendWorkspace
{
  explicit BackendWorkspace(GraphSlamApplicationConfig config)
  : application(new GraphSlamApplication(std::move(config)))
  {
  }

  std::unique_ptr<GraphSlamApplication> application;
};

GraphBasedSlamComponent::Impl::Impl(GraphBasedSlamComponent & node)
: node_(node),
  config_(loadValidatedGraphSlamConfig(node_)),
  clock_(RCL_ROS_TIME),
  tfbuffer_(std::make_shared<rclcpp::Clock>(clock_)),
  listener_(tfbuffer_),
  broadcaster_(&node_),
  backend_(std::make_unique<BackendWorkspace>(makeGraphSlamApplicationConfig(config_))),
  io_ports_(makeExternalIoPorts(node_, config_))
{
  RCLCPP_INFO(node_.get_logger(), "initialization start");

  adjacent_edge_info_weight_ = config_.adjacent_edge_info_weight_;
  adjacent_edge_info_weight_trans_ = config_.adjacent_edge_info_weight_trans_;
  adjacent_edge_info_weight_rot_ = config_.adjacent_edge_info_weight_rot_;

  gnss_origin_accumulator_.configure(
    config_.gnss_origin_min_samples_, config_.gnss_origin_consistency_threshold_m_);
  logGraphSlamConfig(config_, node_.get_logger());
  if (!config_.degeneracy_diagnostics_csv_path_.empty()) {
    degeneracy_csv_enabled_ = io_ports_.map_output->writeBytes(
      config_.degeneracy_diagnostics_csv_path_,
      degeneracy::degeneracyDiagnosticsCsvHeaderLine() + "\n");
    if (!degeneracy_csv_enabled_) {
      io_ports_.diagnostics->emit(
        {ports::DiagnosticLevel::WARNING, "degeneracy_csv_open_failed",
          config_.degeneracy_diagnostics_csv_path_});
    }
  }

  initializePubSub();

  map_save_srv_ = node_.create_service<std_srvs::srv::Empty>(
    "map_save",
    std::bind(
      &GraphBasedSlamComponent::Impl::handleMapSaveRequest,
      this,
      std::placeholders::_1,
      std::placeholders::_2,
      std::placeholders::_3));
}  // NOLINT(readability/fn_size)

GraphBasedSlamComponent::Impl::~Impl() = default;

void GraphBasedSlamComponent::Impl::initializePubSub()
{
  RCLCPP_INFO(node_.get_logger(), "initialize Publishers and Subscribers");

  auto map_array_callback =
    [this](const typename lidarslam_msgs::msg::MapArray::SharedPtr msg_ptr) -> void
    {
      {
        // Preserve input order, but stage expensive conversion and compressed
        // PCD writes outside GraphStateStore's snapshot mutex. Readers keep
        // seeing the previous complete graph until the move-only commit.
        std::lock_guard<std::mutex> ingest_lock(submap_ingest_mtx_);
        lidarslam_msgs::msg::MapArray staged_map = *msg_ptr;
        if (config_.use_pcd_cache_) {
          stageMapArrayCloudCache(staged_map);
        }
        graph_state_.replace(std::move(staged_map));
      }
      runEventDrivenLoopSearch();
    };

  map_array_sub_ =
    node_.create_subscription<lidarslam_msgs::msg::MapArray>(
    "map_array", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(), map_array_callback);

  if (config_.use_odom_input_) {
    // Deep queues so a long loop-search callback cannot overflow the
    // subscription histories and drop frames, plus stamp-based pairing so
    // the submap pose/cloud match is a function of the data, not of the
    // executor schedule. Competitive profiles use exact stamps and reliable
    // delivery; live sensors can retain approximate/best-effort behavior.
    const size_t sync_depth = static_cast<size_t>(std::max(config_.odom_cloud_sync_queue_size_, 1));
    rmw_qos_profile_t odom_qos = rmw_qos_profile_default;
    odom_qos.depth = sync_depth;
    rmw_qos_profile_t cloud_qos = config_.cloud_subscriber_qos_reliable_ ?
      rmw_qos_profile_default : rmw_qos_profile_sensor_data;
    cloud_qos.depth = sync_depth;
    odom_sync_sub_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(
      &node_, "odom_input", odom_qos);
    cloud_sync_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
      &node_, "cloud_input", cloud_qos);
    if (config_.odom_cloud_sync_use_exact_time_) {
      odom_cloud_sync_exact_ =
        std::make_shared<message_filters::Synchronizer<OdomCloudSyncPolicyExact>>(
        OdomCloudSyncPolicyExact(static_cast<uint32_t>(sync_depth)),
        *odom_sync_sub_, *cloud_sync_sub_);
      odom_cloud_sync_exact_->registerCallback(
        std::bind(
          &GraphBasedSlamComponent::Impl::receiveSyncedOdomCloud, this,
          std::placeholders::_1, std::placeholders::_2));
    } else {
      odom_cloud_sync_ = std::make_shared<message_filters::Synchronizer<OdomCloudSyncPolicy>>(
        OdomCloudSyncPolicy(static_cast<uint32_t>(sync_depth)), *odom_sync_sub_, *cloud_sync_sub_);
      odom_cloud_sync_->registerCallback(
        std::bind(
          &GraphBasedSlamComponent::Impl::receiveSyncedOdomCloud, this,
          std::placeholders::_1, std::placeholders::_2));
    }
    RCLCPP_INFO(
      node_.get_logger(),
      "Direct odom+cloud input mode enabled (%s-stamp-synced, queue %zu, cloud %s)",
      config_.odom_cloud_sync_use_exact_time_ ? "exact" : "approximate", sync_depth,
      config_.cloud_subscriber_qos_reliable_ ? "reliable" : "best-effort");
  }

  modified_map_pub_ = node_.create_publisher<sensor_msgs::msg::PointCloud2>(
    "modified_map",
    rclcpp::QoS(10));

  modified_map_array_pub_ = node_.create_publisher<lidarslam_msgs::msg::MapArray>(
    "modified_map_array", rclcpp::QoS(10));

  modified_path_pub_ = node_.create_publisher<nav_msgs::msg::Path>(
    "modified_path",
    rclcpp::QoS(10));

  if (config_.use_imu_preintegration_) {
    auto imu_callback =
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) -> void
      {
        receiveImu(*msg);
      };
    imu_sub_ = node_.create_subscription<sensor_msgs::msg::Imu>(
      "/imu", rclcpp::SensorDataQoS(), imu_callback);
    RCLCPP_INFO(node_.get_logger(), "IMU preintegration enabled, subscribed to /imu");
  }

  if (config_.use_gnss_) {
    gnss_sub_ = node_.create_subscription<sensor_msgs::msg::NavSatFix>(
      config_.gnss_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {receiveNavSatFix(*msg);});
    RCLCPP_INFO(
      node_.get_logger(),
      "GNSS constraints enabled, subscribed to %s",
      config_.gnss_topic_.c_str());
  }

  RCLCPP_INFO(node_.get_logger(), "initialization end");
}

void GraphBasedSlamComponent::Impl::handleMapSaveRequest(
  const MapSaveRequestHeader request_header,
  const MapSaveRequest request,
  const MapSaveResponse response)
{
  static_cast<void>(request_header);
  static_cast<void>(request);
  static_cast<void>(response);

  std::cout << "Received an request to save the map" << std::endl;
  lidarslam_msgs::msg::MapArray map_array_msg;
  if (!snapshotGraphState(map_array_msg)) {
    std::cout << "initial map is not received" << std::endl;
    return;
  }
  doPoseAdjustment(map_array_msg, true);
}

bool GraphBasedSlamComponent::Impl::snapshotGraphState(
  lidarslam_msgs::msg::MapArray & map_array_msg)
{
  return graph_state_.snapshot(map_array_msg);
}

void GraphBasedSlamComponent::Impl::runEventDrivenLoopSearch()
{
  // A component may be hosted by a MultiThreadedExecutor. Elect one callback
  // to own BackendCore and coalesce notifications that arrive while it drains.
  backend_work_drain_.request([this]() {drainEventDrivenLoopSearch();});
}

void GraphBasedSlamComponent::Impl::drainEventDrivenLoopSearch()
{
  while (rclcpp::ok()) {
    lidarslam_msgs::msg::MapArray map_array_msg;
    if (!snapshotGraphState(map_array_msg)) {return;}
    const int num_submaps = static_cast<int>(map_array_msg.submaps.size());
    if (num_submaps < 2) {return;}
    if (backend_->application->stateSnapshot().next_query_index >= num_submaps) {return;}
    if (map_array_msg.cloud_coordinate != map_array_msg.LOCAL) {
      RCLCPP_WARN(node_.get_logger(), "cloud_coordinate should be local, but it's not local.");
    }
    std::vector<backend_core::SubmapMeta> ordered_submaps;
    ordered_submaps.reserve(map_array_msg.submaps.size());
    for (const auto & submap : map_array_msg.submaps) {
      backend_core::SubmapMeta meta;
      tf2::fromMsg(submap.pose, meta.pose);
      meta.travel_distance = submap.distance;
      ordered_submaps.push_back(meta);
    }
    const auto raw_cloud_provider =
      [this, &map_array_msg](int idx) -> pcl::PointCloud<pcl::PointXYZI>::Ptr {
        return loadSubmapCloud(map_array_msg, idx);
      };
    const auto events = backend_->application->processSubmaps(
      ordered_submaps, raw_cloud_provider);
    for (const auto & event : events) {
      if (config_.debug_flag_) {
        RCLCPP_INFO(
          node_.get_logger(), "event-driven loop search, query submap %d of %d", event.query_index,
          num_submaps);
      }
      for (const auto & line : event.search_output.logs) {
        if (line.via_logger) {
          RCLCPP_INFO(node_.get_logger(), "%s", line.text.c_str());
        } else {
          std::cout << line.text << std::endl;
        }
      }
      if (!event.registration_searched && config_.debug_flag_) {
        RCLCPP_INFO(
          node_.get_logger(), "loop search registration skipped for query submap %d by stride %d",
          event.query_index, config_.loop_search_query_stride_);
      }
      if (event.search_output.proposal.found && !event.graph_changed) {
        std::cout << "loop edge skipped as redundant or lower quality" << std::endl;
      }
      if (event.graph_changed) {
        lidarslam_msgs::msg::MapArray query_prefix;
        query_prefix.header = map_array_msg.header;
        query_prefix.cloud_coordinate = map_array_msg.cloud_coordinate;
        query_prefix.submaps.assign(
          map_array_msg.submaps.begin(),
          map_array_msg.submaps.begin() + event.query_index + 1);
        doPoseAdjustment(std::move(query_prefix), config_.use_save_map_in_loop_);
      }
    }
  }
}

void GraphBasedSlamComponent::Impl::doPoseAdjustment(
  lidarslam_msgs::msg::MapArray map_array_msg,
  bool do_save_map)
{
  /* Plain-data inputs for the extracted pose-graph optimization
     (graph_based_slam/pose_graph_optimization.hpp); the IMU/GNSS buffers
     and their matching stay in the node, the g2o assembly does not. */
  int submaps_size = map_array_msg.submaps.size();
  std::vector<pose_graph::SubmapNode> submap_nodes;
  submap_nodes.reserve(submaps_size);
  for (int i = 0; i < submaps_size; i++) {
    Eigen::Affine3d affine;
    Eigen::fromMsg(map_array_msg.submaps[i].pose, affine);
    pose_graph::SubmapNode node;
    node.pose = Eigen::Isometry3d(affine.matrix());
    submap_nodes.push_back(node);
  }

  /* IMU rotation constraints (pre-pass over the IMU buffer) */
  std::vector<pose_graph::ImuRotationConstraint> imu_constraints;
  if (config_.use_imu_preintegration_ && submaps_size > 1) {
    std::lock_guard<std::mutex> imu_lock(imu_mtx_);
    for (int i = 1; i < submaps_size; i++) {
      double t0 = rclcpp::Time(map_array_msg.submaps[i - 1].header.stamp).seconds();
      double t1 = rclcpp::Time(map_array_msg.submaps[i].header.stamp).seconds();
      if (t1 <= t0 || t1 - t0 > 30.0) {continue;}

      Eigen::Quaterniond imu_delta_q = integrateImuRotation(t0, t1);
      if (imu_delta_q.isApprox(Eigen::Quaterniond::Identity(), 1e-8)) {continue;}

      // Relative measurement: translation from odometry, rotation from IMU
      Eigen::Isometry3d odom_relative =
        submap_nodes[i - 1].pose.inverse() * submap_nodes[i].pose;
      pose_graph::ImuRotationConstraint imu_constraint;
      imu_constraint.from = i - 1;
      imu_constraint.to = i;
      imu_constraint.measurement = Eigen::Isometry3d::Identity();
      imu_constraint.measurement.linear() = imu_delta_q.toRotationMatrix();
      imu_constraint.measurement.translation() = odom_relative.translation();
      imu_constraints.push_back(imu_constraint);
    }
    if (config_.debug_flag_) {
      RCLCPP_INFO(
        node_.get_logger(), "Added %zu IMU rotation constraint edges", imu_constraints.size());
    }
  }

  /* GNSS anchors (pre-pass: nearest-measurement match over the buffer) */
  std::vector<pose_graph::GnssConstraint> gnss_constraints;
  if (config_.use_gnss_ && gnss_origin_set_) {
    std::lock_guard<std::mutex> gnss_lock(gnss_mtx_);
    int gnss_rtk_like_edges_added = 0;

    for (int i = 0; i < submaps_size; i++) {
      double submap_time = rclcpp::Time(map_array_msg.submaps[i].header.stamp).seconds();
      // Find nearest GNSS measurement
      double best_dt = std::numeric_limits<double>::max();
      GnssEnu best_gnss;
      bool found = false;
      for (const auto & g : gnss_buffer_) {
        double dt = std::abs(g.stamp - submap_time);
        if (dt < best_dt) {
          best_dt = dt;
          best_gnss = g;
          found = true;
        }
      }
      if (!found || best_dt > 1.0) {continue;}  // Skip if no GNSS within 1 second

      pose_graph::GnssConstraint gnss_constraint;
      gnss_constraint.submap_index = i;
      gnss_constraint.position = Eigen::Vector3d(best_gnss.x, best_gnss.y, best_gnss.z);
      gnss_constraint.info_diag =
        Eigen::Vector3d(best_gnss.info_x, best_gnss.info_y, best_gnss.info_z);
      gnss_constraints.push_back(gnss_constraint);
      if (best_gnss.rtk_like) {
        gnss_rtk_like_edges_added++;
      }
    }
    if (config_.debug_flag_) {
      RCLCPP_INFO(
        node_.get_logger(),
        "Added %zu GNSS position constraint edges (%d RTK-like by covariance)",
        gnss_constraints.size(), gnss_rtk_like_edges_added);
    }
  }

  // GNSS yaw alignment: the odometry frame's x axis is the initial heading,
  // the anchors' x axis is east. Estimate the planar odom->ENU transform,
  // move the whole graph into the ENU frame, and release the vertex-0 gauge
  // so the anchors govern the global pose — without this the anchors shear
  // the map (docs/research/gnss-constraint-first-validation.md).
  bool fix_first_vertex = true;
  if (config_.gnss_align_yaw_ && !gnss_constraints.empty()) {
    std::vector<Eigen::Vector2d> odom_xy;
    std::vector<Eigen::Vector2d> enu_xy;
    odom_xy.reserve(gnss_constraints.size());
    enu_xy.reserve(gnss_constraints.size());
    for (const auto & gnss_constraint : gnss_constraints) {
      const Eigen::Vector3d p = submap_nodes[gnss_constraint.submap_index].pose.translation();
      odom_xy.emplace_back(p.x(), p.y());
      enu_xy.emplace_back(gnss_constraint.position.x(), gnss_constraint.position.y());
    }
    const auto alignment = gnss_alignment::estimatePlanarAlignment(
      odom_xy, enu_xy, config_.gnss_yaw_alignment_min_anchors_,
        config_.gnss_yaw_alignment_min_baseline_m_);
    if (alignment.valid) {
      Eigen::Isometry3d enu_from_odom = Eigen::Isometry3d::Identity();
      enu_from_odom.linear() =
        Eigen::AngleAxisd(alignment.yaw_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
      enu_from_odom.translation() =
        Eigen::Vector3d(alignment.translation.x(), alignment.translation.y(), 0.0);
      for (auto & node : submap_nodes) {
        node.pose = enu_from_odom * node.pose;
      }
      fix_first_vertex = false;
      RCLCPP_INFO(
        node_.get_logger(),
        "GNSS yaw alignment applied: yaw=%.2f deg, baseline=%.1f m, rms=%.2f m; "
        "vertex-0 gauge released, graph moves to the ENU frame",
        alignment.yaw_rad * 180.0 / M_PI, alignment.baseline_m, alignment.rms_residual_m);
    } else if (config_.debug_flag_) {
      RCLCPP_INFO(
        node_.get_logger(),
        "GNSS yaw alignment not applied (pairs=%zu, baseline=%.1f m); keeping the "
        "vertex-0 gauge",
        odom_xy.size(), alignment.baseline_m);
    }
  }

  const PoseGraphConfigBundle pose_graph_config = makePoseGraphConfig(
    config_, adjacent_edge_info_weight_, adjacent_edge_info_weight_trans_,
    adjacent_edge_info_weight_rot_);

  const std::string bundle_pose_graph_path = config_.map_save_dir_ + "/pose_graph.g2o";
  if (do_save_map) {
    io_ports_.map_output->prepareDirectory(config_.map_save_dir_);
  }
  PoseGraphRequest optimization_request;
  optimization_request.submaps = submap_nodes;
  optimization_request.imu_constraints = imu_constraints;
  optimization_request.gnss_constraints = gnss_constraints;
  optimization_request.adjacent_config = pose_graph_config.adjacent;
  optimization_request.loop_config = pose_graph_config.loop;
  optimization_request.imu_config = pose_graph_config.imu;
  optimization_request.chi2_collection = pose_graph_config.chi2_collection;
  optimization_request.fix_first_vertex = fix_first_vertex;
  std::vector<double> artifact_timestamps;
  artifact_timestamps.reserve(map_array_msg.submaps.size());
  for (const auto & submap : map_array_msg.submaps) {
    artifact_timestamps.push_back(rclcpp::Time(submap.header.stamp).seconds());
  }
  const OptimizationArtifacts application_result =
    backend_->application->optimizeAndSerialize(
    optimization_request, artifact_timestamps);
  const pose_graph::OptimizationResult & opt_result = application_result.optimization;

  if (do_save_map &&
    !io_ports_.map_output->writeBytes(bundle_pose_graph_path, opt_result.pose_graph_g2o))
  {
    io_ports_.diagnostics->emit(
      {ports::DiagnosticLevel::WARNING, "pose_graph_write_failed", bundle_pose_graph_path});
  }
  if (!config_.save_pose_graph_path_.empty() &&
    (!do_save_map ||
    std::filesystem::path(config_.save_pose_graph_path_).lexically_normal() !=
    std::filesystem::path(bundle_pose_graph_path).lexically_normal()) &&
    !io_ports_.map_output->writeBytes(
      config_.save_pose_graph_path_, opt_result.pose_graph_g2o))
  {
    io_ports_.diagnostics->emit(
      {ports::DiagnosticLevel::WARNING, "pose_graph_write_failed",
        config_.save_pose_graph_path_});
  }

  if (config_.adjacent_edge_info_auto_scale_ && submaps_size > 1) {
    graphslam::detail::AutoScaleConfig cfg = pose_graph_config.auto_scale;

    if (config_.adjacent_edge_info_auto_scale_split_trans_rot_) {
      // Level 2: split the post-opt residuals into translation / rotation
      // blocks and rescale w_trans and w_rot independently (the chi2
      // vectors come back from optimizePoseGraph in edge order).
      const double median_chi2_trans =
        graphslam::detail::medianChi2(opt_result.adjacent_trans_chi2);
      const double median_chi2_rot =
        graphslam::detail::medianChi2(opt_result.adjacent_rot_chi2);

      cfg.target_nis = config_.adjacent_edge_info_auto_scale_target_nis_trans_;
      const double prev_w_trans = adjacent_edge_info_weight_trans_;
      adjacent_edge_info_weight_trans_ =
        graphslam::detail::nextScale(prev_w_trans, median_chi2_trans, cfg);

      cfg.target_nis = config_.adjacent_edge_info_auto_scale_target_nis_rot_;
      const double prev_w_rot = adjacent_edge_info_weight_rot_;
      adjacent_edge_info_weight_rot_ =
        graphslam::detail::nextScale(prev_w_rot, median_chi2_rot, cfg);

      RCLCPP_INFO(
        node_.get_logger(),
        "[auto_scale_split] trans median_chi2=%.3f target=%.3f w_trans=%.3f -> %.3f | "
        "rot median_chi2=%.3f target=%.3f w_rot=%.3f -> %.3f (n=%zu)",
        median_chi2_trans, config_.adjacent_edge_info_auto_scale_target_nis_trans_,
        prev_w_trans, adjacent_edge_info_weight_trans_,
        median_chi2_rot, config_.adjacent_edge_info_auto_scale_target_nis_rot_,
        prev_w_rot, adjacent_edge_info_weight_rot_,
        opt_result.adjacent_trans_chi2.size());
    } else {
      const double median_chi2 = graphslam::detail::medianChi2(opt_result.adjacent_chi2);

      cfg.target_nis = config_.adjacent_edge_info_auto_scale_target_nis_;
      const double prev_weight = adjacent_edge_info_weight_;
      adjacent_edge_info_weight_ =
        graphslam::detail::nextScale(prev_weight, median_chi2, cfg);

      RCLCPP_INFO(
        node_.get_logger(),
        "[auto_scale] median_chi2=%.3f (n=%zu) target=%.3f weight=%.3f -> %.3f",
        median_chi2, opt_result.adjacent_chi2.size(), cfg.target_nis, prev_weight,
        adjacent_edge_info_weight_);
    }
  }

  /* modified_map publish */
  std::cout << "modified_map publish" << std::endl;
  lidarslam_msgs::msg::MapArray modified_map_array_msg;
  modified_map_array_msg.header = map_array_msg.header;
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  std::vector<TimedSubmapCloud> dynamic_filter_submaps;
  if (do_save_map && config_.use_dynamic_object_filter_) {
    dynamic_filter_submaps.reserve(submaps_size);
  }
  for (int i = 0; i < submaps_size; i++) {
    Eigen::Affine3d se3(opt_result.poses[i].matrix());
    geometry_msgs::msg::Pose pose = tf2::toMsg(se3);

    /* map */
    Eigen::Affine3d previous_affine;
    tf2::fromMsg(map_array_msg.submaps[i].pose, previous_affine);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr = loadSubmapCloud(map_array_msg, i);
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud_ptr(
      new pcl::PointCloud<pcl::PointXYZI>());

    pcl::transformPointCloud(*cloud_ptr, *transformed_cloud_ptr, se3.matrix().cast<float>());
    sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg_ptr(new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(*transformed_cloud_ptr, *cloud_msg_ptr);
    *map_ptr += *transformed_cloud_ptr;
    if (do_save_map && config_.use_dynamic_object_filter_) {
      dynamic_filter_submaps.push_back(
        TimedSubmapCloud{
          i,
          Eigen::Vector3d(se3.translation().x(), se3.translation().y(), se3.translation().z()),
          transformed_cloud_ptr});
    }

    /* submap */
    lidarslam_msgs::msg::SubMap submap;
    submap.header = map_array_msg.submaps[i].header;
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
  if (do_save_map) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_to_save = map_ptr;
    if (config_.use_dynamic_object_filter_) {
      const auto filter_result =
        buildDynamicObjectFilteredMap(dynamic_filter_submaps,
          makeDynamicObjectFilterConfig(config_));
      if (!filter_result.cloud->empty()) {
        map_to_save = filter_result.cloud;
      }
      RCLCPP_INFO(
        node_.get_logger(),
        "Dynamic object filter: input_points %zu, kept %zu/%zu candidate voxels, "
        "removed %zu, always_keep %zu, output_points %zu",
        filter_result.stats.input_points,
        filter_result.stats.kept_candidate_voxels,
        filter_result.stats.candidate_voxels,
        filter_result.stats.removed_candidate_voxels,
        filter_result.stats.always_keep_voxels,
        filter_result.stats.output_points);
    }
    saveGridDividedMap(map_to_save);
    writeMapBundleArtifacts(
      map_array_msg, application_result.artifacts, opt_result.poses);
    writeDegeneracyReport();
  }
}

void GraphBasedSlamComponent::Impl::receiveNavSatFix(const sensor_msgs::msg::NavSatFix & msg)
{
  if (msg.status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
    return;  // No valid fix
  }
  if (!isUsableGnssFix(msg)) {
    return;
  }

  std::lock_guard<std::mutex> lock(gnss_mtx_);

  if (!gnss_origin_set_) {
    tryInitializeGnssOrigin(msg.latitude, msg.longitude, msg.altitude);
    if (!gnss_origin_set_) {
      return;
    }
  }

  Eigen::Vector3d enu = geodeticToEnu(msg.latitude, msg.longitude, msg.altitude);
  const detail::GnssConstraintWeights gnss_weights =
    detail::computeGnssConstraintWeights(msg, makeGnssWeightingConfig(config_));
  const double receive_time_sec = io_ports_.clock->nowSeconds();
  const double header_time_sec = rclcpp::Time(msg.header.stamp).seconds();
  const detail::GnssTimestampResolution stamp_resolution =
    detail::resolveGnssMeasurementStamp(
    header_time_sec, receive_time_sec, config_.gnss_header_stamp_max_skew_sec_);
  GnssEnu g;
  g.stamp = stamp_resolution.stamp_sec;
  g.x = enu.x();
  g.y = enu.y();
  g.z = enu.z();
  g.info_x = gnss_weights.info_x;
  g.info_y = gnss_weights.info_y;
  g.info_z = gnss_weights.info_z;
  g.covariance_valid = gnss_weights.covariance_valid;
  g.rtk_like = gnss_weights.rtk_like;
  g.horizontal_stddev_m = gnss_weights.horizontal_stddev_m;
  gnss_buffer_.push_back(g);

  if (config_.debug_flag_ && stamp_resolution.used_fallback) {
    RCLCPP_WARN_THROTTLE(
      node_.get_logger(),
      *node_.get_clock(),
      5000,
      "GNSS header stamp %.3f s differs from receive time %.3f s by more than "
      "%.3f s; using receive time",
      header_time_sec, receive_time_sec, config_.gnss_header_stamp_max_skew_sec_);
  }

  if (config_.debug_flag_ && gnss_weights.covariance_valid) {
    RCLCPP_INFO_THROTTLE(
      node_.get_logger(),
      *node_.get_clock(),
      5000,
      "GNSS covariance weighting: horizontal_stddev=%.3f m, class=%s, info=(%.3f, %.3f, %.3f)",
      gnss_weights.horizontal_stddev_m,
      gnss_weights.rtk_like ? "rtk_like" : "non_rtk",
      gnss_weights.info_x, gnss_weights.info_y, gnss_weights.info_z);
  }

  // Limit buffer size
  if (gnss_buffer_.size() > 100000) {
    gnss_buffer_.erase(gnss_buffer_.begin(), gnss_buffer_.begin() + 25000);
  }
}

bool GraphBasedSlamComponent::Impl::isUsableGnssFix(const sensor_msgs::msg::NavSatFix & msg) const
{
  return detail::isUsableGeodeticFix(msg.latitude, msg.longitude, msg.altitude);
}

void GraphBasedSlamComponent::Impl::tryInitializeGnssOrigin(double lat, double lon, double alt)
{
  const detail::GnssOriginUpdate update = gnss_origin_accumulator_.add(lat, lon, alt);
  if (update.reset_after_jump) {
    RCLCPP_WARN(
      node_.get_logger(),
      "Resetting GNSS origin initialization after %.1f m jump in candidate fixes",
      update.deviation_m);
  }
  if (update.restarted_after_inconsistency) {
    RCLCPP_WARN(
      node_.get_logger(),
      "GNSS origin candidates were inconsistent (max deviation %.1f m), restarting accumulation",
      update.deviation_m);
  }
  if (!update.initialized) {
    return;
  }

  gnss_origin_lat_ = update.origin.latitude_deg;
  gnss_origin_lon_ = update.origin.longitude_deg;
  gnss_origin_alt_ = update.origin.altitude_m;
  gnss_origin_set_ = true;
  RCLCPP_INFO(
    node_.get_logger(),
    "GNSS origin set from %d consistent fixes: lat=%.8f, lon=%.8f, alt=%.2f",
    config_.gnss_origin_min_samples_, gnss_origin_lat_, gnss_origin_lon_, gnss_origin_alt_);
}

Eigen::Vector3d GraphBasedSlamComponent::Impl::geodeticToEnu(
  double lat, double lon, double alt) const
{
  const detail::GeodeticOrigin origin {
    gnss_origin_lat_, gnss_origin_lon_, gnss_origin_alt_};
  return detail::geodeticToEnu(lat, lon, alt, origin);
}

void GraphBasedSlamComponent::Impl::receiveImu(const sensor_msgs::msg::Imu & msg)
{
  std::lock_guard<std::mutex> lock(imu_mtx_);
  StampedImu imu;
  imu.stamp = rclcpp::Time(msg.header.stamp).seconds();
  imu.gx = msg.angular_velocity.x;
  imu.gy = msg.angular_velocity.y;
  imu.gz = msg.angular_velocity.z;
  imu.ax = msg.linear_acceleration.x;
  imu.ay = msg.linear_acceleration.y;
  imu.az = msg.linear_acceleration.z;
  imu.qx = msg.orientation.x;
  imu.qy = msg.orientation.y;
  imu.qz = msg.orientation.z;
  imu.qw = msg.orientation.w;
  imu_buffer_.push_back(imu);
  if (imu_buffer_.size() > kMaxImuBufferSize) {
    imu_buffer_.erase(imu_buffer_.begin(), imu_buffer_.begin() + kMaxImuBufferSize / 4);
  }
}

Eigen::Quaterniond GraphBasedSlamComponent::Impl::integrateImuRotation(double t0, double t1) const
{
  // Integrate gyroscope measurements between t0 and t1
  Eigen::Quaterniond delta_q = Eigen::Quaterniond::Identity();

  // Find first IMU sample >= t0
  auto it = std::lower_bound(
    imu_buffer_.begin(), imu_buffer_.end(), t0,
    [](const StampedImu & imu, double t) {return imu.stamp < t;});

  if (it == imu_buffer_.end()) {
    return delta_q;  // no data
  }

  double prev_t = t0;
  for (; it != imu_buffer_.end() && it->stamp <= t1; ++it) {
    double dt = it->stamp - prev_t;
    if (dt <= 0.0 || dt > 0.5) {
      prev_t = it->stamp;
      continue;
    }
    // Small angle quaternion integration
    Eigen::Vector3d omega(it->gx, it->gy, it->gz);
    double angle = omega.norm() * dt;
    if (angle > 1e-10) {
      Eigen::Quaterniond dq(Eigen::AngleAxisd(angle, omega.normalized()));
      delta_q = delta_q * dq;
      delta_q.normalize();
    }
    prev_t = it->stamp;
  }

  return delta_q;
}

void GraphBasedSlamComponent::Impl::receiveSyncedOdomCloud(
  const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg)
{
  Eigen::Vector3d pos(
    odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y,
    odom_msg->pose.pose.position.z);
  if (!std::isfinite(pos.x()) || !std::isfinite(pos.y()) || !std::isfinite(pos.z())) {
    return;
  }
  if (config_.debug_flag_ && !first_synced_input_logged_) {
    first_synced_input_logged_ = true;
    RCLCPP_INFO(
      node_.get_logger(), "First synced odom+cloud pair: (%.2f, %.2f, %.2f), %zu bytes", pos.x(),
      pos.y(), pos.z(), cloud_msg->data.size());
  }
  recordScanDegeneracy(*odom_msg);
  tryCreateSubmap(*odom_msg, *cloud_msg);
}

void GraphBasedSlamComponent::Impl::recordScanDegeneracy(const nav_msgs::msg::Odometry & odom_msg)
{
  // Opt-in, report-only (v0.8 Phase 1): compute nothing at all unless one of
  // the two diagnostics outputs was requested, so the default path stays
  // byte-for-byte identical in behavior and cost.
  const bool csv_enabled = degeneracy_csv_enabled_;
  if (!csv_enabled && !config_.save_degeneracy_report_) {
    return;
  }

  const degeneracy::CovarianceLocalizabilityResult result =
    degeneracy::analyzeOdometryCovariance(odom_msg.pose.covariance);
  const double stamp_sec = rclcpp::Time(odom_msg.header.stamp).seconds();

  std::lock_guard<std::mutex> lock(degeneracy_mtx_);
  if (csv_enabled) {
    io_ports_.map_output->appendBytes(
      config_.degeneracy_diagnostics_csv_path_,
      degeneracy::degeneracyDiagnosticsCsvRowLine(stamp_sec, result) + "\n");
  }
  if (config_.save_degeneracy_report_) {
    degeneracy_accumulator_.add(stamp_sec, result);
  }
}

void GraphBasedSlamComponent::Impl::writeDegeneracyReport()
{
  if (!config_.save_degeneracy_report_) {
    return;
  }
  // Best-effort, same as the other map-bundle artifacts
  // (map_projector_info.yaml etc.): a failure to write the report must not
  // fail the map save itself.
  degeneracy::DegeneracyReportSummary summary;
  {
    std::lock_guard<std::mutex> lock(degeneracy_mtx_);
    summary = degeneracy_accumulator_.summary();
  }
  const std::string report_path = config_.map_save_dir_ + "/degeneracy_report.yaml";
  const std::vector<std::string> lines = degeneracy::degeneracyReportYamlLines(summary);
  std::ostringstream report;
  for (size_t i = 0; i < lines.size(); ++i) {
    report << lines[i] << "\n";
  }
  if (!io_ports_.map_output->writeBytes(report_path, report.str())) {
    io_ports_.diagnostics->emit(
      {ports::DiagnosticLevel::WARNING, "degeneracy_report_write_failed", report_path});
    return;
  }
  std::cout << "Degeneracy report: " << report_path << std::endl;
}

void GraphBasedSlamComponent::Impl::tryCreateSubmap(
  const nav_msgs::msg::Odometry & odom_msg,
  const sensor_msgs::msg::PointCloud2 & cloud_msg)
{
  std::unique_lock<std::mutex> ingest_lock(submap_ingest_mtx_);
  Eigen::Vector3d pos(
    odom_msg.pose.pose.position.x,
    odom_msg.pose.pose.position.y,
    odom_msg.pose.pose.position.z);

  // Check distance threshold (semantics pinned by test_submap_creation.cpp)
  const submap_creation::Decision decision = submap_creation::evaluate(
    pos, last_submap_position_valid_, last_submap_position_, config_.submap_distance_threshold_);
  if (!decision.create) {return;}
  accumulated_distance_ += decision.distance;
  last_submap_position_ = pos;
  last_submap_position_valid_ = true;

  // Create SubMap (use "map" frame for SLAM output regardless of odom frame)
  lidarslam_msgs::msg::SubMap submap;
  submap.header.stamp = odom_msg.header.stamp;
  submap.header.frame_id = "map";
  submap.distance = accumulated_distance_;
  submap.pose = odom_msg.pose.pose;
  submap.cloud = cloud_msg;
  submap.cloud.header.frame_id = odom_msg.child_frame_id;

  const int submap_idx = static_cast<int>(graph_state_.submapCount());
  if (config_.use_pcd_cache_) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = convertSubmapCloud(submap.cloud);
    if (io_ports_.submap_storage->store(submap_idx, *cloud)) {
      submap.cloud = sensor_msgs::msg::PointCloud2();
    } else {
      io_ports_.diagnostics->emit(
        {ports::DiagnosticLevel::WARNING, "submap_cache_write_failed",
          "failed to store submap " + std::to_string(submap_idx)});
    }
  }
  std_msgs::msg::Header map_header;
  map_header.stamp = odom_msg.header.stamp;
  map_header.frame_id = "map";
  const int n = static_cast<int>(graph_state_.append(std::move(submap), map_header));
  ingest_lock.unlock();

  if (n % 50 == 0) {
    RCLCPP_INFO(
      node_.get_logger(), "Odom input: %d submaps, distance: %.1fm", n,
      accumulated_distance_);
  }

  runEventDrivenLoopSearch();
}

GraphBasedSlamComponent::Impl::PointCloudPtr
GraphBasedSlamComponent::Impl::convertSubmapCloud(
  const sensor_msgs::msg::PointCloud2 & cloud_msg)
{
  PointCloudPtr cloud(new PointCloud);
  const bool has_intensity =
    pointcloud_conversion::fromRosMsgWithOptionalIntensity(cloud_msg, *cloud);
  if (!has_intensity) {
    RCLCPP_INFO_ONCE(
      node_.get_logger(),
      "[submap-intensity-defaulted] The deskewed submap cloud has no intensity field; "
      "continuing with intensity=0. Geometry mapping remains available, but exported "
      "intensity is unavailable. No action is needed for geometry-only mapping.");
  }
  return cloud;
}

void GraphBasedSlamComponent::Impl::stageMapArrayCloudCache(
  lidarslam_msgs::msg::MapArray & map_array_msg)
{
  std::vector<ports::SubmapWrite> writes;
  for (int i = 0; i < static_cast<int>(map_array_msg.submaps.size()); ++i) {
    const auto & submap = map_array_msg.submaps[i];
    if (submap.cloud.data.empty()) {
      continue;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = convertSubmapCloud(submap.cloud);
    if (!cloud->empty()) {
      writes.push_back({i, cloud});
    }
  }
  const std::vector<bool> stored = io_ports_.submap_storage->storeBatch(writes);
  for (std::size_t i = 0; i < writes.size(); ++i) {
    if (i < stored.size() && stored[i]) {
      map_array_msg.submaps[writes[i].index].cloud = sensor_msgs::msg::PointCloud2();
    } else {
      io_ports_.diagnostics->emit(
        {ports::DiagnosticLevel::WARNING, "submap_cache_write_failed",
          "failed to store submap " + std::to_string(writes[i].index)});
    }
  }
}

pcl::PointCloud<pcl::PointXYZI>::Ptr GraphBasedSlamComponent::Impl::loadSubmapCloud(
  const lidarslam_msgs::msg::MapArray & map_array_msg,
  int idx)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
  if (config_.use_pcd_cache_) {
    cloud = io_ports_.submap_storage->load(idx);
    if (!cloud->empty()) {
      return cloud;
    }
  }
  cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
  if (idx >= 0 && idx < static_cast<int>(map_array_msg.submaps.size())) {
    cloud = convertSubmapCloud(map_array_msg.submaps[idx].cloud);
  }
  return cloud;
}

void GraphBasedSlamComponent::Impl::writeMapBundleArtifacts(
  const lidarslam_msgs::msg::MapArray & map_array_msg,
  const DeterministicArtifacts & artifacts,
  const std::vector<Eigen::Isometry3d> & optimized_poses)
{
  io_ports_.map_output->prepareDirectory(config_.map_save_dir_);

  const std::string trajectory_path = config_.map_save_dir_ + "/trajectory_optimized.tum";
  if (!io_ports_.map_output->writeBytes(
      trajectory_path, artifacts.trajectory_optimized_tum))
  {
    io_ports_.diagnostics->emit(
      {ports::DiagnosticLevel::WARNING, "trajectory_write_failed", trajectory_path});
  }

  const std::string loop_edges_path = config_.map_save_dir_ + "/loop_edges.csv";
  if (!io_ports_.map_output->writeBytes(loop_edges_path, artifacts.loop_edges_csv)) {
    io_ports_.diagnostics->emit(
      {ports::DiagnosticLevel::WARNING, "loop_edges_write_failed", loop_edges_path});
  }

  map_saver::BundleManifest manifest;
  manifest.frame_id = map_array_msg.header.frame_id.empty() ? "map" : map_array_msg.header.frame_id;
  manifest.submap_count = optimized_poses.size();
  manifest.loop_edge_count = artifacts.loop_edge_count;
  manifest.map_leaf_size = config_.map_leaf_size_;
  manifest.grid_size_x = config_.map_grid_size_x_;
  manifest.grid_size_y = config_.map_grid_size_y_;
  manifest.dynamic_object_filter = config_.use_dynamic_object_filter_;
  manifest.planar_map_filter = config_.use_planar_map_filter_;
  manifest.planar_map_filter_voxel_size = config_.planar_map_filter_voxel_size_;
  manifest.planar_map_filter_min_neighbors = config_.planar_map_filter_min_neighbors_;
  manifest.planar_map_filter_max_small_eigenvalue_ratio =
    config_.planar_map_filter_max_small_eigenvalue_ratio_;
  manifest.planar_map_filter_min_middle_eigenvalue_ratio =
    config_.planar_map_filter_min_middle_eigenvalue_ratio_;
  manifest.planar_map_filter_min_retained_ratio = config_.planar_map_filter_min_retained_ratio_;
  const std::string manifest_path = config_.map_save_dir_ + "/map_bundle.yaml";
  if (!io_ports_.map_output->writeBytes(
      manifest_path, map_saver::bundleManifestYaml(manifest)))
  {
    io_ports_.diagnostics->emit(
      {ports::DiagnosticLevel::WARNING, "manifest_write_failed", manifest_path});
  }

  RCLCPP_INFO(
    node_.get_logger(), "Map bundle artifacts: trajectory=%s loops=%s manifest=%s",
    trajectory_path.c_str(), loop_edges_path.c_str(), manifest_path.c_str());
}

void GraphBasedSlamComponent::Impl::saveGridDividedMap(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr & map)
{
  if (map->empty()) {
    std::cout << "Map is empty, skipping save." << std::endl;
    return;
  }

  // Create output directory (clean existing PCD files to prevent orphans)
  std::string out_dir = config_.map_save_dir_ + "/pointcloud_map";
  io_ports_.map_output->prepareDirectory(out_dir);
  io_ports_.map_output->removeByExtension(out_dir, {".pcd", ".yaml"});

  // Downsample the map
  pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::VoxelGrid<pcl::PointXYZI> vg;
  vg.setInputCloud(map);
  vg.setLeafSize(config_.map_leaf_size_, config_.map_leaf_size_, config_.map_leaf_size_);
  vg.filter(*downsampled);

  std::cout << map_saver::downsampleLogLine(map->size(), downsampled->size(),
      config_.map_leaf_size_)
            << std::endl;

  // Apply map-quality refinement to the same leaf-sized point set that is
  // exported and evaluated. Filtering before this VoxelGrid changes local
  // covariance according to raw submap sampling density and does not match
  // the saved-map contract.
  if (config_.use_planar_map_filter_) {
    const auto filter_result =
      buildPlanarMapFilteredMap(downsampled, makePlanarMapFilterConfig(config_));
    downsampled = filter_result.cloud;
    RCLCPP_INFO(
      node_.get_logger(),
      "Planar map filter: input_points %zu, finite %zu, planar_voxels %zu/%zu, "
      "supported_points %zu, output_points %zu, fallback %s",
      filter_result.stats.input_points,
      filter_result.stats.finite_points,
      filter_result.stats.planar_voxels,
      filter_result.stats.voxel_count,
      filter_result.stats.supported_points,
      filter_result.stats.output_points,
      filter_result.stats.fallback_to_input ? "true" : "false");
  }

  // Compute bounding box and grid-aligned bounds (semantics pinned by
  // test_map_saver.cpp)
  pcl::PointXYZI min_pt, max_pt;
  pcl::getMinMax3D(*downsampled, min_pt, max_pt);

  const map_saver::GridConfig grid_config = makeGridConfig(config_);
  const map_saver::GridBounds grid_bounds =
    map_saver::computeGridBounds(min_pt.x, min_pt.y, max_pt.x, max_pt.y, grid_config);

  // Assign points to grid cells
  std::map<std::pair<int, int>, pcl::PointCloud<pcl::PointXYZI>::Ptr> grid_cells;
  for (const auto & pt : downsampled->points) {
    auto key = map_saver::cellIndexFor(pt.x, pt.y, grid_bounds, grid_config);
    if (grid_cells.find(key) == grid_cells.end()) {
      grid_cells[key] = pcl::PointCloud<pcl::PointXYZI>::Ptr(
        new pcl::PointCloud<pcl::PointXYZI>);
    }
    grid_cells[key]->push_back(pt);
  }

  // Save each grid cell as PCD and build the Autoware pointcloud_map_loader
  // metadata (content produced in map_saver.hpp)
  std::ostringstream meta;
  meta << map_saver::metadataHeader(grid_config);

  int saved = 0;
  for (auto & [key, cloud] : grid_cells) {
    if (cloud->empty()) {continue;}
    const map_saver::CellFile cell = map_saver::makeCellFile(key, grid_bounds, grid_config);
    io_ports_.map_output->writePointCloud(out_dir + "/" + cell.filename, *cloud);
    meta << map_saver::metadataEntry(cell);
    saved++;
  }
  io_ports_.map_output->writeBytes(out_dir + "/pointcloud_map_metadata.yaml", meta.str());

  // Also save the full map as a single PCD for convenience
  io_ports_.map_output->writePointCloud(config_.map_save_dir_ + "/map.pcd", *downsampled);

  std::cout << map_saver::savedMapLogLine(saved, grid_config, out_dir) << std::endl;
  std::cout << "Total points: " << downsampled->size() << std::endl;
  std::cout << "Metadata: " << out_dir << "/pointcloud_map_metadata.yaml" << std::endl;

  // Always emit map_projector_info.yaml so Autoware can load pointcloud-only maps.
  std::string proj_file = config_.map_save_dir_ + "/map_projector_info.yaml";
  io_ports_.map_output->writeBytes(
    proj_file,
    map_saver::projectorInfoYaml(gnss_origin_set_, gnss_origin_lat_, gnss_origin_lon_));
  std::cout << map_saver::projectorInfoLogLine(gnss_origin_set_, proj_file) << std::endl;
}
}  // namespace graphslam
