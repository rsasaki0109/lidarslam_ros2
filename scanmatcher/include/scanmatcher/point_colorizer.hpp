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

#ifndef SCANMATCHER__POINT_COLORIZER_HPP_
#define SCANMATCHER__POINT_COLORIZER_HPP_

// Pure, ROS-/OpenCV-free core for colouring a SLAM point cloud from posed
// camera images online. It is the streaming counterpart of the offline Python
// colorizer (tools/gaussian_splatting/pointcloud_io.colorize_by_projection_robust):
// project a map point into the current camera, reject it if a nearer point in
// the same image cell occludes it (a coarse per-frame z-buffer), sample the
// pixel (nearest or bilinear), and fold the observation into a depth-weighted
// running mean so the nearest — highest-resolution, least-foreshortened —
// views dominate the final colour.
//
// The realtime ScanMatcher/colorizer node is the shell around this: it syncs
// image + camera_info, looks up the lidar->camera extrinsic via tf, and calls
// projectPoint / FrameZBuffer / sampleBilinear / PointColor::add per keyframe.
// Keeping the maths here (Eigen only) lets it be unit-tested without a running
// ROS graph, matching pose_prediction.hpp / imu_processing.hpp in this package.

#include <Eigen/Core>

#include <cstdint>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace graphslam
{
namespace point_colorizer
{

// Pinhole intrinsics plus image size (pixels). Matches the K / width / height a
// sensor_msgs/CameraInfo carries.
struct CameraIntrinsics
{
  float fx {0.0f};
  float fy {0.0f};
  float cx {0.0f};
  float cy {0.0f};
  int width {0};
  int height {0};
};

// OpenCV/ROS plumb_bob distortion coefficients (k1, k2, p1, p2, k3).
// Applying these during projection lets us sample a raw, distorted image
// directly; a zero-initialised value is equivalent to an ideal pinhole lens.
struct PlumbBobDistortion
{
  float k1 {0.0f};
  float k2 {0.0f};
  float p1 {0.0f};
  float p2 {0.0f};
  float k3 {0.0f};
};

// A borrowed, row-major image buffer (no ownership) — a thin stand-in for a
// cv::Mat so the core needs no OpenCV. `channels` is 1 (grey) or >=3 (RGB[A]);
// only the first three channels are read. `row_stride` is bytes per row.
struct ImageView
{
  const std::uint8_t * data {nullptr};
  int width {0};
  int height {0};
  int channels {0};
  int row_stride {0};

  bool valid() const
  {
    return data != nullptr && width > 0 && height > 0 && channels >= 1 &&
           row_stride >= width * channels;
  }

  // Channel `c` of pixel (x, y); grey images broadcast their single channel.
  std::uint8_t at(int x, int y, int c) const
  {
    const int cc = (channels >= 3) ? c : 0;
    return data[static_cast<std::size_t>(y) * row_stride + x * channels + cc];
  }
};

// Project a world/map point into the camera. `world_to_cam` is the OpenCV
// convention (+z forward, world->camera) so depth is the camera-frame z. Fills
// (u, v) in pixels and `depth` in metres; returns true only when the point is
// in front of the camera and lands inside the image.
inline bool projectPoint(
  const CameraIntrinsics & intr, const Eigen::Matrix4f & world_to_cam,
  const Eigen::Vector3f & p_world, float & u, float & v, float & depth,
  const PlumbBobDistortion * distortion = nullptr)
{
  const Eigen::Vector3f cam =
    world_to_cam.block<3, 3>(0, 0) * p_world + world_to_cam.block<3, 1>(0, 3);
  depth = cam.z();
  if (!(depth > 1e-6f)) {
    return false;
  }
  float x = cam.x() / depth;
  float y = cam.y() / depth;
  if (distortion != nullptr) {
    const float x2 = x * x;
    const float y2 = y * y;
    const float xy = x * y;
    const float r2 = x2 + y2;
    const float radial =
      1.0f + distortion->k1 * r2 + distortion->k2 * r2 * r2 +
      distortion->k3 * r2 * r2 * r2;
    const float xd = x * radial + 2.0f * distortion->p1 * xy +
      distortion->p2 * (r2 + 2.0f * x2);
    const float yd = y * radial + distortion->p1 * (r2 + 2.0f * y2) +
      2.0f * distortion->p2 * xy;
    x = xd;
    y = yd;
  }
  u = intr.fx * x + intr.cx;
  v = intr.fy * y + intr.cy;
  return u >= 0.0f && u <= static_cast<float>(intr.width - 1) &&
         v >= 0.0f && v <= static_cast<float>(intr.height - 1);
}

// True when (u, v) sits at least `margin` pixels inside every image border.
// Lens vignetting darkens the border band in a way per-frame exposure gains
// cannot repair, so colour sampling can skip it while the z-buffer (occlusion
// geometry, valid to the border) still uses the full frame — the streaming
// counterpart of the offline `image_margin` option. `margin <= 0` keeps the
// full frame.
inline bool insideImageMargin(
  const CameraIntrinsics & intr, float u, float v, int margin)
{
  if (margin <= 0) {
    return true;
  }
  const float m = static_cast<float>(margin);
  return u >= m && u <= static_cast<float>(intr.width - 1) - m &&
         v >= m && v <= static_cast<float>(intr.height - 1) - m;
}

// Nearest-pixel colour sample into `rgb` (3 elements). Coordinates are clamped
// to the image, so callers may pass any in-bounds (u, v).
inline void sampleNearest(
  const ImageView & img, float u, float v, float rgb[3])
{
  const int x = std::min(std::max(static_cast<int>(std::lround(u)), 0), img.width - 1);
  const int y = std::min(std::max(static_cast<int>(std::lround(v)), 0), img.height - 1);
  for (int c = 0; c < 3; ++c) {
    rgb[c] = static_cast<float>(img.at(x, y, c));
  }
}

// Bilinear colour sample into `rgb` (3 elements): blends the four surrounding
// pixels (edges clamp, never wrap), cutting the colour bleed nearest sampling
// leaves along edges. Mirrors the Python `interp='bilinear'` path.
inline void sampleBilinear(
  const ImageView & img, float u, float v, float rgb[3])
{
  const float uc = std::min(std::max(u, 0.0f), static_cast<float>(img.width - 1));
  const float vc = std::min(std::max(v, 0.0f), static_cast<float>(img.height - 1));
  const int x0 = static_cast<int>(std::floor(uc));
  const int y0 = static_cast<int>(std::floor(vc));
  const int x1 = std::min(x0 + 1, img.width - 1);
  const int y1 = std::min(y0 + 1, img.height - 1);
  const float wx = uc - static_cast<float>(x0);
  const float wy = vc - static_cast<float>(y0);
  for (int c = 0; c < 3; ++c) {
    const float top =
      static_cast<float>(img.at(x0, y0, c)) * (1.0f - wx) +
      static_cast<float>(img.at(x1, y0, c)) * wx;
    const float bot =
      static_cast<float>(img.at(x0, y1, c)) * (1.0f - wx) +
      static_cast<float>(img.at(x1, y1, c)) * wx;
    rgb[c] = top * (1.0f - wy) + bot * wy;
  }
}

// Coarse per-frame z-buffer over `bin`-pixel cells. Built from the points that
// project into this frame, it lets the colorizer skip a point whose depth sits
// well behind the nearest surface in its cell — the online equivalent of the
// offline z-buffer occlusion test, so points behind a wall do not steal the
// wall's colour.
class FrameZBuffer
{
public:
  FrameZBuffer(const CameraIntrinsics & intr, int bin)
  : bin_(std::max(bin, 1)),
    cols_((intr.width + bin_ - 1) / bin_),
    rows_((intr.height + bin_ - 1) / bin_),
    buf_(static_cast<std::size_t>(cols_) * rows_,
      std::numeric_limits<float>::infinity())
  {
  }

  // Record a projected point's depth in its cell (keeps the nearest).
  void insert(float u, float v, float depth)
  {
    const std::size_t idx = cell(u, v);
    if (depth < buf_[idx]) {
      buf_[idx] = depth;
    }
  }

  // Visible if within `tol` (+2 % of range) of the nearest depth in its cell.
  bool visible(float u, float v, float depth, float tol) const
  {
    return depth <= buf_[cell(u, v)] + tol + 0.02f * depth;
  }

  float nearestDepth(float u, float v) const {return buf_[cell(u, v)];}

private:
  std::size_t cell(float u, float v) const
  {
    int cx = static_cast<int>(u) / bin_;
    int cy = static_cast<int>(v) / bin_;
    cx = std::min(std::max(cx, 0), cols_ - 1);
    cy = std::min(std::max(cy, 0), rows_ - 1);
    return static_cast<std::size_t>(cy) * cols_ + cx;
  }

  int bin_;
  int cols_;
  int rows_;
  std::vector<float> buf_;
};

// Per-point colour accumulator: a depth-weighted running mean (weight
// 1/(depth+eps)) so nearer, sharper observations dominate — the streaming form
// of the offline `prefer_near`. O(1) memory per point, no sample buffer.
struct PointColor
{
  double sum_w {0.0};
  double sum_wr {0.0};
  double sum_wg {0.0};
  double sum_wb {0.0};
  float best_depth {std::numeric_limits<float>::infinity()};
  std::uint16_t count {0};

  void add(const float rgb[3], float depth)
  {
    const double w = 1.0 / (static_cast<double>(depth) + 1e-3);
    sum_w += w;
    sum_wr += w * rgb[0];
    sum_wg += w * rgb[1];
    sum_wb += w * rgb[2];
    if (depth < best_depth) {
      best_depth = depth;
    }
    if (count < std::numeric_limits<std::uint16_t>::max()) {
      ++count;
    }
  }

  bool seen() const {return count > 0;}

  // Colour backed by at least `min_count` accumulated observations. Colours
  // confirmed only once or twice are usually occlusion-fringe or specular
  // one-offs that pepper flat surfaces; consumers can require more before
  // trusting the mean (min_count <= 1 degrades to `seen`).
  bool confirmed(std::uint16_t min_count) const
  {
    return count >= std::max<std::uint16_t>(min_count, 1);
  }

  // Rounded [0,255] mean colour into `out` (3 elements). Undefined if unseen.
  void mean(std::uint8_t out[3]) const
  {
    const double inv = (sum_w > 0.0) ? 1.0 / sum_w : 0.0;
    const double vals[3] = {sum_wr * inv, sum_wg * inv, sum_wb * inv};
    for (int c = 0; c < 3; ++c) {
      const double v = std::min(std::max(vals[c] + 0.5, 0.0), 255.0);
      out[c] = static_cast<std::uint8_t>(v);
    }
  }
};

// Median luminance (Rec.601) of an ImageView — the exposure statistic the shell
// uses to rescale each frame toward a global median before accumulation, so
// auto-exposure swings do not bias the colour. Returns 0 for an empty image.
inline float medianLuminance(const ImageView & img)
{
  if (!img.valid()) {
    return 0.0f;
  }
  std::vector<float> lum;
  lum.reserve(static_cast<std::size_t>(img.width) * img.height);
  for (int y = 0; y < img.height; ++y) {
    for (int x = 0; x < img.width; ++x) {
      lum.push_back(
        0.299f * img.at(x, y, 0) + 0.587f * img.at(x, y, 1) +
        0.114f * img.at(x, y, 2));
    }
  }
  const std::size_t mid = lum.size() / 2;
  std::nth_element(lum.begin(), lum.begin() + mid, lum.end());
  return lum[mid];
}

}  // namespace point_colorizer
}  // namespace graphslam

#endif  // SCANMATCHER__POINT_COLORIZER_HPP_
