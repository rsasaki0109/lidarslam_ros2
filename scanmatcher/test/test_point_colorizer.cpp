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

#include <Eigen/Core>
#include <gtest/gtest.h>

#include <cstdint>

#include <vector>

#include "scanmatcher/point_colorizer.hpp"

namespace
{
using graphslam::point_colorizer::CameraIntrinsics;
using graphslam::point_colorizer::FrameZBuffer;
using graphslam::point_colorizer::ImageView;
using graphslam::point_colorizer::PointColor;
using graphslam::point_colorizer::PlumbBobDistortion;
using graphslam::point_colorizer::medianLuminance;
using graphslam::point_colorizer::projectPoint;
using graphslam::point_colorizer::sampleBilinear;
using graphslam::point_colorizer::sampleNearest;

// 100x100 pinhole, principal point at the centre, unit focal scale of 100.
CameraIntrinsics makeCam()
{
  CameraIntrinsics intr;
  intr.fx = 100.0f;
  intr.fy = 100.0f;
  intr.cx = 50.0f;
  intr.cy = 50.0f;
  intr.width = 100;
  intr.height = 100;
  return intr;
}

// A backing RGB buffer wrapped as an ImageView (row-major, 3 channels).
struct RgbImage
{
  int width;
  int height;
  std::vector<std::uint8_t> data;

  RgbImage(int w, int h)
  : width(w), height(h), data(static_cast<std::size_t>(w) * h * 3, 0)
  {
  }

  void set(int x, int y, std::uint8_t r, std::uint8_t g, std::uint8_t b)
  {
    const std::size_t i = (static_cast<std::size_t>(y) * width + x) * 3;
    data[i] = r;
    data[i + 1] = g;
    data[i + 2] = b;
  }

  ImageView view() const
  {
    ImageView v;
    v.data = data.data();
    v.width = width;
    v.height = height;
    v.channels = 3;
    v.row_stride = width * 3;
    return v;
  }
};

}  // namespace

// --------------------------------------------------------------------------- //
// projectPoint
// --------------------------------------------------------------------------- //
TEST(PointColorizer, ProjectsOnAxisPointToPrincipalPoint)
{
  const CameraIntrinsics intr = makeCam();
  const Eigen::Matrix4f w2c = Eigen::Matrix4f::Identity();
  float u = 0.0f;
  float v = 0.0f;
  float depth = 0.0f;
  ASSERT_TRUE(projectPoint(intr, w2c, Eigen::Vector3f(0.0f, 0.0f, 5.0f), u, v, depth));
  EXPECT_NEAR(u, 50.0f, 1e-4f);
  EXPECT_NEAR(v, 50.0f, 1e-4f);
  EXPECT_NEAR(depth, 5.0f, 1e-4f);
}

TEST(PointColorizer, RejectsPointBehindCamera)
{
  const CameraIntrinsics intr = makeCam();
  const Eigen::Matrix4f w2c = Eigen::Matrix4f::Identity();
  float u = 0.0f;
  float v = 0.0f;
  float depth = 0.0f;
  EXPECT_FALSE(projectPoint(intr, w2c, Eigen::Vector3f(0.0f, 0.0f, -5.0f), u, v, depth));
}

TEST(PointColorizer, RejectsOutOfFramePoint)
{
  const CameraIntrinsics intr = makeCam();
  const Eigen::Matrix4f w2c = Eigen::Matrix4f::Identity();
  float u = 0.0f;
  float v = 0.0f;
  float depth = 0.0f;
  // u = 100*10/5 + 50 = 250 -> off image.
  EXPECT_FALSE(projectPoint(intr, w2c, Eigen::Vector3f(10.0f, 0.0f, 5.0f), u, v, depth));
}

TEST(PointColorizer, HonoursCameraExtrinsic)
{
  const CameraIntrinsics intr = makeCam();
  // Camera translated so a point 2 m ahead sits at depth 5 (offset of 3).
  Eigen::Matrix4f w2c = Eigen::Matrix4f::Identity();
  w2c(2, 3) = 3.0f;
  float u = 0.0f;
  float v = 0.0f;
  float depth = 0.0f;
  ASSERT_TRUE(projectPoint(intr, w2c, Eigen::Vector3f(0.0f, 0.0f, 2.0f), u, v, depth));
  EXPECT_NEAR(depth, 5.0f, 1e-4f);
}

TEST(PointColorizer, AppliesPlumbBobRadialDistortion)
{
  CameraIntrinsics intr = makeCam();
  intr.width = 200;
  const Eigen::Matrix4f w2c = Eigen::Matrix4f::Identity();
  PlumbBobDistortion distortion;
  distortion.k1 = 0.2f;
  float u = 0.0f;
  float v = 0.0f;
  float depth = 0.0f;
  ASSERT_TRUE(projectPoint(
    intr, w2c, Eigen::Vector3f(1.0f, 0.0f, 2.0f), u, v, depth, &distortion));
  // x=0.5, r^2=0.25 -> xd=0.5*(1 + 0.2*0.25)=0.525.
  EXPECT_NEAR(u, 102.5f, 1e-4f);
  EXPECT_NEAR(v, 50.0f, 1e-4f);
}

TEST(PointColorizer, AppliesPlumbBobTangentialDistortion)
{
  CameraIntrinsics intr = makeCam();
  intr.width = 200;
  intr.height = 200;
  const Eigen::Matrix4f w2c = Eigen::Matrix4f::Identity();
  PlumbBobDistortion distortion;
  distortion.p1 = 0.1f;
  distortion.p2 = 0.05f;
  float u = 0.0f;
  float v = 0.0f;
  float depth = 0.0f;
  ASSERT_TRUE(projectPoint(
    intr, w2c, Eigen::Vector3f(0.4f, 0.2f, 2.0f), u, v, depth, &distortion));
  // x=.2, y=.1, r^2=.05: xd=.2 + .004 + .0065; yd=.1 + .007 + .002.
  EXPECT_NEAR(u, 71.05f, 1e-4f);
  EXPECT_NEAR(v, 60.9f, 1e-4f);
}

// --------------------------------------------------------------------------- //
// Sampling: nearest vs bilinear
// --------------------------------------------------------------------------- //
TEST(PointColorizer, BilinearBlendsNeighbouringPixels)
{
  RgbImage img(100, 100);
  img.set(50, 50, 100, 100, 100);
  img.set(51, 50, 200, 200, 200);
  float rgb[3] = {0.0f, 0.0f, 0.0f};
  // 40 % of the way from pixel 50 to 51 -> 0.6*100 + 0.4*200 = 140.
  sampleBilinear(img.view(), 50.4f, 50.0f, rgb);
  for (int c = 0; c < 3; ++c) {
    EXPECT_NEAR(rgb[c], 140.0f, 1e-3f);
  }
  // Nearest snaps to pixel 50 -> the un-blended 100.
  float near_rgb[3] = {0.0f, 0.0f, 0.0f};
  sampleNearest(img.view(), 50.4f, 50.0f, near_rgb);
  for (int c = 0; c < 3; ++c) {
    EXPECT_NEAR(near_rgb[c], 100.0f, 1e-3f);
  }
}

TEST(PointColorizer, BilinearClampsAtBorder)
{
  RgbImage img(100, 100);
  img.set(99, 99, 10, 20, 30);
  float rgb[3] = {0.0f, 0.0f, 0.0f};
  // The +1 neighbour is out of range; clamping must keep it in-bounds.
  sampleBilinear(img.view(), 99.0f, 99.0f, rgb);
  EXPECT_NEAR(rgb[0], 10.0f, 1e-3f);
  EXPECT_NEAR(rgb[1], 20.0f, 1e-3f);
  EXPECT_NEAR(rgb[2], 30.0f, 1e-3f);
}

// --------------------------------------------------------------------------- //
// FrameZBuffer occlusion
// --------------------------------------------------------------------------- //
TEST(PointColorizer, ZBufferHidesOccludedPoint)
{
  const CameraIntrinsics intr = makeCam();
  FrameZBuffer zbuf(intr, 4);
  // A near surface and a far point land in the same cell (~pixel 50,50).
  zbuf.insert(50.0f, 50.0f, 2.0f);
  zbuf.insert(50.0f, 50.0f, 8.0f);
  EXPECT_TRUE(zbuf.visible(50.0f, 50.0f, 2.0f, 0.15f));
  EXPECT_FALSE(zbuf.visible(50.0f, 50.0f, 8.0f, 0.15f));
}

TEST(PointColorizer, ZBufferKeepsCoplanarPointsVisible)
{
  const CameraIntrinsics intr = makeCam();
  FrameZBuffer zbuf(intr, 4);
  zbuf.insert(50.0f, 50.0f, 5.0f);
  zbuf.insert(51.0f, 51.0f, 5.05f);
  // Both within tol of the cell's nearest depth -> both visible.
  EXPECT_TRUE(zbuf.visible(50.0f, 50.0f, 5.0f, 0.15f));
  EXPECT_TRUE(zbuf.visible(51.0f, 51.0f, 5.05f, 0.15f));
}

// --------------------------------------------------------------------------- //
// PointColor depth-weighted mean (streaming prefer_near)
// --------------------------------------------------------------------------- //
TEST(PointColorizer, MeanIsUnseenBeforeAnyObservation)
{
  PointColor pc;
  EXPECT_FALSE(pc.seen());
  EXPECT_EQ(pc.count, 0);
}

TEST(PointColorizer, AveragesRepeatedObservations)
{
  PointColor pc;
  const float a[3] = {10.0f, 20.0f, 30.0f};
  const float b[3] = {30.0f, 40.0f, 50.0f};
  pc.add(a, 5.0f);
  pc.add(b, 5.0f);  // equal depth -> equal weight -> plain mean
  ASSERT_TRUE(pc.seen());
  EXPECT_EQ(pc.count, 2);
  std::uint8_t out[3] = {0, 0, 0};
  pc.mean(out);
  EXPECT_EQ(out[0], 20);
  EXPECT_EQ(out[1], 30);
  EXPECT_EQ(out[2], 40);
}

TEST(PointColorizer, ImageMarginSkipsBorderBand)
{
  using graphslam::point_colorizer::insideImageMargin;
  CameraIntrinsics intr;
  intr.fx = intr.fy = 100.0f;
  intr.cx = intr.cy = 50.0f;
  intr.width = intr.height = 100;
  // Centre passes any sane margin; a point 4 px from the border fails a
  // 10 px margin but passes with the margin disabled.
  EXPECT_TRUE(insideImageMargin(intr, 50.0f, 50.0f, 10));
  EXPECT_FALSE(insideImageMargin(intr, 95.0f, 50.0f, 10));
  EXPECT_FALSE(insideImageMargin(intr, 50.0f, 4.0f, 10));
  EXPECT_TRUE(insideImageMargin(intr, 95.0f, 50.0f, 0));
  EXPECT_TRUE(insideImageMargin(intr, 95.0f, 50.0f, -3));
  // Boundary is inclusive at exactly `margin` pixels from the border.
  EXPECT_TRUE(insideImageMargin(intr, 10.0f, 89.0f, 10));
}

TEST(PointColorizer, ConfirmedRequiresMinimumSamples)
{
  PointColor pc;
  const float grey[3] = {100.0f, 100.0f, 100.0f};
  EXPECT_FALSE(pc.confirmed(1));
  pc.add(grey, 5.0f);
  EXPECT_TRUE(pc.confirmed(1));
  EXPECT_TRUE(pc.confirmed(0));  // degrades to seen()
  EXPECT_FALSE(pc.confirmed(3));
  pc.add(grey, 5.0f);
  pc.add(grey, 5.0f);
  EXPECT_TRUE(pc.confirmed(3));
}

TEST(PointColorizer, NearObservationDominatesFarOne)
{
  PointColor pc;
  const float near_green[3] = {0.0f, 200.0f, 0.0f};
  const float far_red[3] = {200.0f, 0.0f, 0.0f};
  pc.add(near_green, 1.0f);   // weight ~1.0
  pc.add(far_red, 50.0f);     // weight ~0.02
  std::uint8_t out[3] = {0, 0, 0};
  pc.mean(out);
  // The near green view dominates; red stays small.
  EXPECT_GT(out[1], 180);
  EXPECT_LT(out[0], 20);
  EXPECT_EQ(pc.best_depth, 1.0f);
}

// --------------------------------------------------------------------------- //
// medianLuminance (exposure statistic)
// --------------------------------------------------------------------------- //
TEST(PointColorizer, MedianLuminanceOfFlatImage)
{
  RgbImage img(10, 10);
  for (int y = 0; y < 10; ++y) {
    for (int x = 0; x < 10; ++x) {
      img.set(x, y, 60, 60, 60);
    }
  }
  EXPECT_NEAR(medianLuminance(img.view()), 60.0f, 1e-3f);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
