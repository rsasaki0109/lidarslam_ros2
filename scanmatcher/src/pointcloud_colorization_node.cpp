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

// Realtime point-cloud colorization node.
//
// The ScanMatcher registration core runs on pcl::PointXYZI (no colour); this
// node keeps that core untouched and colours the map in a decoupled layer. It
// caches the latest SLAM map (a PointXYZI PointCloud2), and for every camera
// image (synchronised with its CameraInfo) it looks up the map->camera_optical
// transform via tf, projects the map points, rejects the occluded ones with a
// coarse per-frame z-buffer, samples the pixel (bilinear), exposure-normalises
// the frame, and folds each observation into a depth-weighted running mean per
// voxel. A timer publishes the accumulated colours as a PointXYZRGB cloud.
//
// All the projection / occlusion / accumulation maths lives in the pure,
// unit-tested scanmatcher/point_colorizer.hpp; this file is only the ROS shell
// (topics, tf, message_filters sync, PointCloud2 <-> voxel-store plumbing). No
// cv_bridge/OpenCV: sensor_msgs/Image already exposes a raw row-major buffer,
// which wraps straight into point_colorizer::ImageView.
//
// The camera frame must be the optical frame (REP-103: +z forward, +x right,
// +y down), matching the OpenCV convention projectPoint expects.

#include <Eigen/Core>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cstdint>

#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <tf2_eigen/tf2_eigen.hpp>

#include "scanmatcher/point_colorizer.hpp"

namespace graphslam
{

using point_colorizer::CameraIntrinsics;
using point_colorizer::FrameZBuffer;
using point_colorizer::ImageView;
using point_colorizer::PointColor;
using point_colorizer::PlumbBobDistortion;

class PointCloudColorizationNode : public rclcpp::Node
{
public:
  PointCloudColorizationNode()
  : Node("pointcloud_colorization"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    map_frame_ = declare_parameter("map_frame", std::string("map"));
    camera_optical_frame_ =
      declare_parameter("camera_optical_frame", std::string("camera_optical_link"));
    const std::string map_topic = declare_parameter("map_topic", std::string("map"));
    const std::string image_topic =
      declare_parameter("image_topic", std::string("image_raw"));
    const std::string camera_info_topic =
      declare_parameter("camera_info_topic", std::string("camera_info"));
    const std::string colored_map_topic =
      declare_parameter("colored_map_topic", std::string("colored_map"));

    voxel_size_ = declare_parameter("voxel_size", 0.1);
    zbuf_bin_ = static_cast<int>(declare_parameter("zbuf_bin", 4));
    depth_tol_ = declare_parameter("depth_tolerance", 0.15);
    image_margin_ = static_cast<int>(declare_parameter("image_margin", 0));
    min_color_samples_ = static_cast<int>(declare_parameter("min_color_samples", 1));
    use_bilinear_ = declare_parameter("use_bilinear", true);
    normalize_exposure_ = declare_parameter("normalize_exposure", true);
    exposure_ema_alpha_ = declare_parameter("exposure_ema_alpha", 0.1);
    tf_timeout_ = declare_parameter("tf_timeout", 0.1);
    const double publish_period = declare_parameter("publish_period", 2.0);
    const int sync_queue = static_cast<int>(declare_parameter("sync_queue_size", 10));
    max_project_points_ =
      static_cast<int>(declare_parameter("max_project_points", 2000000));

    // Sensor streams (map/image/camera_info) are typically best-effort; a
    // best-effort subscriber is compatible with both best-effort publishers
    // (rosbag2, live drivers) and reliable ones (a latched SLAM /map), so use
    // it everywhere to avoid silently dropping every message on a QoS mismatch.
    map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      map_topic, rclcpp::SensorDataQoS(),
      std::bind(&PointCloudColorizationNode::mapCallback, this, std::placeholders::_1));

    image_sub_.subscribe(this, image_topic, rmw_qos_profile_sensor_data);
    info_sub_.subscribe(this, camera_info_topic, rmw_qos_profile_sensor_data);
    sync_ = std::make_shared<Sync>(SyncPolicy(sync_queue), image_sub_, info_sub_);
    sync_->registerCallback(std::bind(
      &PointCloudColorizationNode::imageCallback, this,
      std::placeholders::_1, std::placeholders::_2));

    colored_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(colored_map_topic, rclcpp::QoS(1));

    publish_timer_ = create_wall_timer(
      std::chrono::duration<double>(publish_period),
      std::bind(&PointCloudColorizationNode::publishColored, this));

    RCLCPP_INFO(
      get_logger(),
      "colorization: map=%s image=%s info=%s -> %s (voxel=%.3f, bilinear=%d)",
      map_topic.c_str(), image_topic.c_str(), camera_info_topic.c_str(),
      colored_map_topic.c_str(), voxel_size_, static_cast<int>(use_bilinear_));
  }

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;

  struct Voxel
  {
    float x {0.0f};
    float y {0.0f};
    float z {0.0f};
    PointColor color;
  };

  // Quantise a point to a voxel key (three 21-bit signed fields in a 64-bit int).
  std::int64_t voxelKey(float x, float y, float z) const
  {
    const auto q = [this](float v) {
        return static_cast<std::int64_t>(std::floor(v / voxel_size_));
      };
    const std::int64_t mask = (1LL << 21) - 1;
    return ((q(x) & mask) << 42) | ((q(y) & mask) << 21) | (q(z) & mask);
  }

  void mapCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    std::vector<Eigen::Vector3f> pts;
    pts.reserve(msg->width * msg->height);
    sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iy(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iz(*msg, "z");
    for (; ix != ix.end(); ++ix, ++iy, ++iz) {
      const float x = *ix, y = *iy, z = *iz;
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      pts.emplace_back(x, y, z);
    }
    std::lock_guard<std::mutex> lock(mtx_);
    map_points_.swap(pts);
    // Seed geometry so far-unseen points still appear (grey) in the output.
    for (const auto & p : map_points_) {
      auto & vox = voxels_[voxelKey(p.x(), p.y(), p.z())];
      vox.x = p.x();
      vox.y = p.y();
      vox.z = p.z();
    }
  }

  // Wrap a sensor_msgs/Image into an ImageView; returns false for encodings we
  // do not decode. Sets bgr when the channel order is B,G,R (swapped on sample).
  static bool wrapImage(
    const sensor_msgs::msg::Image & img, ImageView & view, bool & bgr)
  {
    const std::string & enc = img.encoding;
    int channels = 0;
    bgr = false;
    if (enc == "rgb8") {
      channels = 3;
    } else if (enc == "bgr8") {
      channels = 3;
      bgr = true;
    } else if (enc == "rgba8") {
      channels = 4;
    } else if (enc == "bgra8") {
      channels = 4;
      bgr = true;
    } else if (enc == "mono8") {
      channels = 1;
    } else {
      return false;
    }
    view.data = img.data.data();
    view.width = static_cast<int>(img.width);
    view.height = static_cast<int>(img.height);
    view.channels = channels;
    view.row_stride = static_cast<int>(img.step);
    return view.valid();
  }

  void imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr image,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr info)
  {
    ImageView view;
    bool bgr = false;
    if (!wrapImage(*image, view, bgr)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "unsupported image encoding '%s'", image->encoding.c_str());
      return;
    }

    CameraIntrinsics intr;
    intr.fx = static_cast<float>(info->k[0]);
    intr.fy = static_cast<float>(info->k[4]);
    intr.cx = static_cast<float>(info->k[2]);
    intr.cy = static_cast<float>(info->k[5]);
    intr.width = static_cast<int>(info->width);
    intr.height = static_cast<int>(info->height);
    if (intr.fx <= 0.0f || intr.fy <= 0.0f || intr.width <= 0 || intr.height <= 0) {
      return;
    }

    PlumbBobDistortion distortion;
    const PlumbBobDistortion * distortion_ptr = nullptr;
    if (info->distortion_model.empty() || info->distortion_model == "plumb_bob") {
      if (info->d.size() >= 4) {
        distortion.k1 = static_cast<float>(info->d[0]);
        distortion.k2 = static_cast<float>(info->d[1]);
        distortion.p1 = static_cast<float>(info->d[2]);
        distortion.p2 = static_cast<float>(info->d[3]);
        if (info->d.size() >= 5) {
          distortion.k3 = static_cast<float>(info->d[4]);
        }
        distortion_ptr = &distortion;
      }
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 30000,
        "unsupported distortion model '%s'; using pinhole projection",
        info->distortion_model.c_str());
    }

    // map -> camera_optical at the image stamp: p_cam = T * p_map.
    Eigen::Matrix4f world_to_cam;
    try {
      const auto tf = tf_buffer_.lookupTransform(
        camera_optical_frame_, map_frame_, image->header.stamp,
        rclcpp::Duration::from_seconds(tf_timeout_));
      world_to_cam = tf2::transformToEigen(tf).matrix().cast<float>();
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "tf lookup failed: %s", ex.what());
      return;
    }

    float exposure_scale = 1.0f;
    if (normalize_exposure_) {
      const float med = point_colorizer::medianLuminance(view);
      if (med > 1e-3f) {
        if (exposure_target_ <= 0.0f) {
          exposure_target_ = med;
        } else {
          exposure_target_ = static_cast<float>(
            exposure_ema_alpha_ * med + (1.0 - exposure_ema_alpha_) * exposure_target_);
        }
        exposure_scale = exposure_target_ / med;
      }
    }

    std::lock_guard<std::mutex> lock(mtx_);
    if (map_points_.empty()) {
      return;
    }
    const int stride = std::max<int>(
      1, static_cast<int>(map_points_.size()) / std::max(1, max_project_points_));

    // Pass 1: build the per-frame z-buffer from the projected points.
    FrameZBuffer zbuf(intr, zbuf_bin_);
    for (std::size_t i = 0; i < map_points_.size(); i += stride) {
      float u, v, depth;
      if (projectPoint(intr, world_to_cam, map_points_[i], u, v, depth, distortion_ptr)) {
        zbuf.insert(u, v, depth);
      }
    }

    // Pass 2: colour the visible points into their voxels.
    for (std::size_t i = 0; i < map_points_.size(); i += stride) {
      const Eigen::Vector3f & p = map_points_[i];
      float u, v, depth;
      if (!projectPoint(intr, world_to_cam, p, u, v, depth, distortion_ptr)) {
        continue;
      }
      if (!zbuf.visible(u, v, depth, static_cast<float>(depth_tol_))) {
        continue;
      }
      if (!point_colorizer::insideImageMargin(intr, u, v, image_margin_)) {
        continue;  // vignette band: geometry above still occludes, colour skips
      }
      float rgb[3];
      if (use_bilinear_) {
        point_colorizer::sampleBilinear(view, u, v, rgb);
      } else {
        point_colorizer::sampleNearest(view, u, v, rgb);
      }
      if (bgr) {
        std::swap(rgb[0], rgb[2]);
      }
      for (int c = 0; c < 3; ++c) {
        rgb[c] = std::min(std::max(rgb[c] * exposure_scale, 0.0f), 255.0f);
      }
      voxels_[voxelKey(p.x(), p.y(), p.z())].color.add(rgb, depth);
    }
  }

  void publishColored()
  {
    if (colored_pub_->get_subscription_count() == 0) {
      return;
    }
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      cloud.reserve(voxels_.size());
      for (const auto & kv : voxels_) {
        const Voxel & vox = kv.second;
        pcl::PointXYZRGB p;
        p.x = vox.x;
        p.y = vox.y;
        p.z = vox.z;
        if (vox.color.confirmed(static_cast<std::uint16_t>(min_color_samples_))) {
          std::uint8_t rgb[3];
          vox.color.mean(rgb);
          p.r = rgb[0];
          p.g = rgb[1];
          p.b = rgb[2];
        } else {
          p.r = p.g = p.b = 128;  // grey for geometry not yet coloured
        }
        cloud.push_back(p);
      }
    }
    if (cloud.empty()) {
      return;
    }
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.frame_id = map_frame_;
    msg.header.stamp = now();
    colored_pub_->publish(msg);
  }

  // Parameters.
  std::string map_frame_;
  std::string camera_optical_frame_;
  double voxel_size_ {0.1};
  int zbuf_bin_ {4};
  double depth_tol_ {0.15};
  int image_margin_ {0};
  int min_color_samples_ {1};
  bool use_bilinear_ {true};
  bool normalize_exposure_ {true};
  double exposure_ema_alpha_ {0.1};
  double tf_timeout_ {0.1};
  int max_project_points_ {2000000};

  // State.
  float exposure_target_ {-1.0f};
  std::mutex mtx_;
  std::vector<Eigen::Vector3f> map_points_;
  std::unordered_map<std::int64_t, Voxel> voxels_;

  // ROS interfaces.
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> info_sub_;
  std::shared_ptr<Sync> sync_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr colored_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace graphslam

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<graphslam::PointCloudColorizationNode>());
  rclcpp::shutdown();
  return 0;
}
