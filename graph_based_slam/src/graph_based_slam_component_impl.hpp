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

#ifndef GRAPH_BASED_SLAM_COMPONENT_IMPL_HPP_
#define GRAPH_BASED_SLAM_COMPONENT_IMPL_HPP_

#include "graph_based_slam/graph_based_slam_component.h"
#include "graph_slam_config.hpp"

#include <Eigen/Core>  // NOLINT(build/include_order)
#include <Eigen/Geometry>  // NOLINT(build/include_order)
#include <pcl/point_cloud.h>  // NOLINT(build/include_order)
#include <pcl/point_types.h>  // NOLINT(build/include_order)
#include <tf2_ros/buffer.h>  // NOLINT(build/include_order)
#include <tf2_ros/transform_broadcaster.h>  // NOLINT(build/include_order)
#include <tf2_ros/transform_listener.h>  // NOLINT(build/include_order)

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <lidarslam_msgs/msg/map_array.hpp>
#include <message_filters/subscriber.h>  // NOLINT(build/include_order)
#include <message_filters/sync_policies/approximate_time.h>  // NOLINT(build/include_order)
#include <message_filters/sync_policies/exact_time.h>  // NOLINT(build/include_order)
#include <message_filters/synchronizer.h>  // NOLINT(build/include_order)
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/empty.hpp>
#include "graph_based_slam/degeneracy_report_summary.hpp"
#include "graph_based_slam/external_io_ports.hpp"
#include "graph_based_slam/gnss_origin_accumulator.hpp"
#include "graph_based_slam/graph_state_store.hpp"
#include "graph_based_slam/loop_edge_set.hpp"
#include "graph_based_slam/serialized_work_drain.hpp"

namespace graphslam
{
struct DeterministicArtifacts;

class GraphBasedSlamComponent::Impl    // NOLINT(runtime/indentation_namespace)
{
public:
  explicit Impl(GraphBasedSlamComponent & node);
  ~Impl();

private:
  struct BackendWorkspace;

  GraphBasedSlamComponent & node_;
  const GraphSlamConfig config_;
  rclcpp::Clock clock_;
  tf2_ros::Buffer tfbuffer_;
  tf2_ros::TransformListener listener_;
  tf2_ros::TransformBroadcaster broadcaster_;

    // Compiler firewall for the ROS-free backend, registration engine,
    // voxel filter and 3D-BBS verifier. Their PCL/pclomp/descriptor headers
    // stay in the component implementation translation unit.
  std::unique_ptr<BackendWorkspace> backend_;
  ports::ExternalIoPorts io_ports_;
    // BackendWorkspace is single-threaded by contract. Keep that contract
    // when hosted by a MultiThreadedExecutor, while coalescing arrivals
    // during a long search.
  scheduling::SerializedWorkDrain backend_work_drain_;
    // Serializes ordered input staging (cloud conversion / PCD writes) but
    // never blocks GraphStateStore snapshots during that I/O.
  std::mutex submap_ingest_mtx_;
  GraphStateStore graph_state_;

  // Adaptive optimizer state is intentionally separate from the immutable
  // startup intent represented by GraphSlamConfig.
  double adjacent_edge_info_weight_ {1000.0};
  double adjacent_edge_info_weight_trans_ {1000.0};
  double adjacent_edge_info_weight_rot_ {1000.0};

  rclcpp::Subscription<lidarslam_msgs::msg::MapArray>::SharedPtr map_array_sub_;
  rclcpp::Publisher<lidarslam_msgs::msg::MapArray>::SharedPtr modified_map_array_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr modified_path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr modified_map_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr map_save_srv_;

  using PointCloud = pcl::PointCloud<pcl::PointXYZI>;
  using PointCloudPtr = PointCloud::Ptr;
  using MapSaveRequestHeader = std::shared_ptr<rmw_request_id_t>;
  using MapSaveRequest = std::shared_ptr<std_srvs::srv::Empty::Request>;
  using MapSaveResponse = std::shared_ptr<std_srvs::srv::Empty::Response>;

  void initializePubSub();
  void handleMapSaveRequest(
    const MapSaveRequestHeader request_header,
    const MapSaveRequest request,
    const MapSaveResponse response);
    // Event-driven scheduling (docs/roadmap/v0.6.md, Phase 2): drain every
    // submap not yet used as a loop-search query, in arrival order, each
    // against exactly the map state [0..q] so the result is a function of
    // the data, not of how the executor batched arrivals.
  void runEventDrivenLoopSearch();
  void drainEventDrivenLoopSearch();
  bool snapshotGraphState(lidarslam_msgs::msg::MapArray & map_array_msg);
  void doPoseAdjustment(
    lidarslam_msgs::msg::MapArray map_array_msg,
    bool do_save_map);
  void publishMapAndPose();

    // Event-driven loop search (the only scheduling semantics since v0.7
    // Phase 0): loop search runs once per submap arrival in arrival order,
    // each query seeing exactly the map state up to itself. The legacy
    // wall-clock timer path and the retired deterministic_loop_scheduling
    // parameter (v0.4 D1) are gone.
  int previous_submaps_num_ {0};

  // PCD disk cache for memory-efficient submap storage.
  void stageMapArrayCloudCache(lidarslam_msgs::msg::MapArray & map_array_msg);
  PointCloudPtr convertSubmapCloud(
    const sensor_msgs::msg::PointCloud2 & cloud_msg);
  pcl::PointCloud<pcl::PointXYZI>::Ptr loadSubmapCloud(
    const lidarslam_msgs::msg::MapArray & map_array_msg, int idx);

    // Autoware-compatible grid-divided PCD map output
  void saveGridDividedMap(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr & map);
  void writeMapBundleArtifacts(
    const lidarslam_msgs::msg::MapArray & map_array_msg,
    const DeterministicArtifacts & artifacts,
    const std::vector<Eigen::Isometry3d> & optimized_poses);

    // Direct odometry + cloud input mode (for LIO frontends). The two
    // streams are stamp-synchronized (message_filters ApproximateTime) so
    // the submap pose/cloud pairing no longer depends on executor timing
    // (v0.6 Phase 1; see docs/research/determinism-variance-attribution.md).
  std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> odom_sync_sub_;
  std::shared_ptr<message_filters::Subscriber<
      sensor_msgs::msg::PointCloud2>> cloud_sync_sub_;
  using OdomCloudSyncPolicy = message_filters::sync_policies::ApproximateTime<
    nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2>;
  std::shared_ptr<message_filters::Synchronizer<OdomCloudSyncPolicy>> odom_cloud_sync_;
  using OdomCloudSyncPolicyExact = message_filters::sync_policies::ExactTime<
    nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2>;
  std::shared_ptr<message_filters::Synchronizer<OdomCloudSyncPolicyExact>>
  odom_cloud_sync_exact_;
  Eigen::Vector3d last_submap_position_ {0, 0, 0};
  bool last_submap_position_valid_ {false};
  double accumulated_distance_ {0.0};
  bool first_synced_input_logged_ {false};
  void receiveSyncedOdomCloud(
    const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg);
  void tryCreateSubmap(
    const nav_msgs::msg::Odometry & odom_msg,
    const sensor_msgs::msg::PointCloud2 & cloud_msg);

    // v0.8 Phase 1 (docs/roadmap/v0.8.md §5): opt-in, report-only per-scan
    // degeneracy diagnostics on the use_odom_input path. The received
    // Odometry pose.covariance (filled anisotropically by the
    // Thirdparty/rko_lio diagnostic patch) is classified per scan via
    // localizability_analysis.hpp; when degeneracy_diagnostics_csv_path is
    // non-empty a per-scan CSV row is appended, and when
    // save_degeneracy_report is true a degeneracy_report.yaml summary joins
    // the map bundle at /map_save time (best-effort, like the other bundle
    // artifacts). Both default off: default behavior (and determinism) is
    // unchanged, and nothing here feeds back into any pose or edge weight.
  std::mutex degeneracy_mtx_;
  bool degeneracy_csv_enabled_ {false};
  degeneracy::DegeneracyReportAccumulator degeneracy_accumulator_;
  void recordScanDegeneracy(const nav_msgs::msg::Odometry & odom_msg);
  void writeDegeneracyReport();

    // GNSS constraints for georeferenced mapping
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;
  struct GnssEnu
  {
    double stamp;
    double x;
    double y;
    double z;    // ENU coordinates relative to origin
    double info_x;
    double info_y;
    double info_z;
    bool covariance_valid;
    bool rtk_like;
    double horizontal_stddev_m;
  };
  std::vector<GnssEnu> gnss_buffer_;
  detail::GnssOriginAccumulator gnss_origin_accumulator_;
  std::mutex gnss_mtx_;
  bool gnss_origin_set_ {false};
  double gnss_origin_lat_ {0.0};
  double gnss_origin_lon_ {0.0};
  double gnss_origin_alt_ {0.0};
  void receiveNavSatFix(const sensor_msgs::msg::NavSatFix & msg);
  bool isUsableGnssFix(const sensor_msgs::msg::NavSatFix & msg) const;
  void tryInitializeGnssOrigin(double lat, double lon, double alt);
  Eigen::Vector3d geodeticToEnu(double lat, double lon, double alt) const;

    // IMU preintegration
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  struct StampedImu
  {
    double stamp;
    double ax;
    double ay;
    double az;
    double gx;
    double gy;
    double gz;
    double qx;
    double qy;
    double qz;
    double qw;
  };
  std::vector<StampedImu> imu_buffer_;
  std::mutex imu_mtx_;
  static constexpr size_t kMaxImuBufferSize = 50000;
  void receiveImu(const sensor_msgs::msg::Imu & msg);
  Eigen::Quaterniond integrateImuRotation(double t0, double t1) const;
};
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM_COMPONENT_IMPL_HPP_
