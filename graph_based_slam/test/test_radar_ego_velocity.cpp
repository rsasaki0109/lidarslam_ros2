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
//    copyright notice, this list of conditions and the following disclaimer
//    in the documentation and/or other materials provided with the distribution.
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

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cmath>
#include <random>
#include <vector>

#include "rko_lio/core/radar_ego_velocity.hpp"

namespace
{

using rko_lio::core::estimate_radar_ego_velocity;
using rko_lio::core::RadarDopplerMeasurement;
using rko_lio::core::RadarEgoVelocityConfig;
using rko_lio::core::RadarEgoVelocityResult;

// Deterministic, well-spread unit directions (Fibonacci sphere); independent of any RNG so
// the RANSAC minimal-sample randomness is the only source of nondeterminism under test.
std::vector<Eigen::Vector3d> fibonacciSphereDirections(const std::size_t count)
{
  std::vector<Eigen::Vector3d> directions;
  directions.reserve(count);
  const double pi = std::acos(-1.0);
  const double golden_angle = pi * (3.0 - std::sqrt(5.0));
  for (std::size_t i = 0; i < count; ++i) {
    const double y = 1.0 - (2.0 * static_cast<double>(i)) / static_cast<double>(count - 1);
    const double radius = std::sqrt(std::max(0.0, 1.0 - y * y));
    const double theta = golden_angle * static_cast<double>(i);
    directions.emplace_back(std::cos(theta) * radius, y, std::sin(theta) * radius);
  }
  return directions;
}

std::vector<RadarDopplerMeasurement> makeInlierMeasurements(
  const Eigen::Vector3d & true_velocity, const double doppler_sign, const std::size_t count,
  const double noise_stddev, std::mt19937 & rng)
{
  std::vector<RadarDopplerMeasurement> measurements;
  measurements.reserve(count);
  std::normal_distribution<double> noise(0.0, noise_stddev);
  for (const auto & direction : fibonacciSphereDirections(count)) {
    RadarDopplerMeasurement measurement;
    measurement.direction = direction;
    measurement.doppler_velocity = doppler_sign * direction.dot(true_velocity) + noise(rng);
    measurements.push_back(measurement);
  }
  return measurements;
}

}  // namespace

TEST(RadarEgoVelocity, RecoversKnownVelocityAndRejectsDynamicOutliers)
{
  const Eigen::Vector3d true_velocity(2.0, -0.4, 0.05);
  const double doppler_sign = -1.0;
  std::mt19937 rng(42);
  std::vector<RadarDopplerMeasurement> measurements =
    makeInlierMeasurements(true_velocity, doppler_sign, 60, 0.01, rng);
  const std::size_t clean_inlier_count = measurements.size();

  // Dynamic-object outliers: same directions as real static points, wildly different Doppler.
  std::uniform_real_distribution<double> outlier_offset(3.0, 6.0);
  for (std::size_t i = 0; i < 8; ++i) {
    RadarDopplerMeasurement outlier;
    outlier.direction = measurements[i].direction;
    outlier.doppler_velocity = measurements[i].doppler_velocity + outlier_offset(rng);
    measurements.push_back(outlier);
  }

  RadarEgoVelocityConfig config;
  config.doppler_sign = doppler_sign;
  config.ransac_iterations = 200;
  config.ransac_inlier_threshold = 0.1;
  config.min_inliers = 8;

  const RadarEgoVelocityResult result = estimate_radar_ego_velocity(measurements, config);

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.velocity.isApprox(true_velocity, 0.02));
  // A couple of noisy inliers may fall just outside the threshold; the outliers must not.
  EXPECT_GE(result.inlier_count, clean_inlier_count - 2);
  EXPECT_LT(result.inlier_count, measurements.size());
}

TEST(RadarEgoVelocity, TooFewPointsIsInvalid)
{
  const RadarDopplerMeasurement a{Eigen::Vector3d(1.0, 0.0, 0.0), 1.0};
  const RadarDopplerMeasurement b{Eigen::Vector3d(0.0, 1.0, 0.0), 1.0};
  const RadarEgoVelocityResult result = estimate_radar_ego_velocity({a, b});
  EXPECT_FALSE(result.valid);
}

TEST(RadarEgoVelocity, TooFewInliersIsInvalid)
{
  std::mt19937 rng(7);
  // Every point is an independent, unrelated dynamic object: no consistent rigid-body fit exists.
  std::vector<RadarDopplerMeasurement> measurements;
  std::uniform_real_distribution<double> velocity_dist(-5.0, 5.0);
  for (const auto & direction : fibonacciSphereDirections(20)) {
    RadarDopplerMeasurement measurement;
    measurement.direction = direction;
    measurement.doppler_velocity = velocity_dist(rng);
    measurements.push_back(measurement);
  }
  RadarEgoVelocityConfig config;
  config.min_inliers = 15;
  const RadarEgoVelocityResult result = estimate_radar_ego_velocity(measurements, config);
  EXPECT_FALSE(result.valid);
}

TEST(RadarEgoVelocity, ZeroDopplerSignIsInvalid)
{
  std::mt19937 rng(1);
  const std::vector<RadarDopplerMeasurement> measurements =
    makeInlierMeasurements(Eigen::Vector3d(1.0, 0.0, 0.0), -1.0, 20, 0.0, rng);
  RadarEgoVelocityConfig config;
  config.doppler_sign = 0.0;
  const RadarEgoVelocityResult result = estimate_radar_ego_velocity(measurements, config);
  EXPECT_FALSE(result.valid);
}

TEST(RadarEgoVelocity, SameInputsAreBitwiseDeterministic)
{
  std::mt19937 rng(99);
  const std::vector<RadarDopplerMeasurement> measurements =
    makeInlierMeasurements(Eigen::Vector3d(1.5, 0.3, -0.2), -1.0, 40, 0.02, rng);
  RadarEgoVelocityConfig config;
  const RadarEgoVelocityResult first = estimate_radar_ego_velocity(measurements, config);
  const RadarEgoVelocityResult second = estimate_radar_ego_velocity(measurements, config);
  ASSERT_TRUE(first.valid);
  ASSERT_TRUE(second.valid);
  EXPECT_EQ(first.velocity, second.velocity);
  EXPECT_EQ(first.inlier_count, second.inlier_count);
}
