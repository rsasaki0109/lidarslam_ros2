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

#include <limits>

#include "rko_lio/core/radar_ego_velocity.hpp"

namespace
{

using rko_lio::core::blend_icp_radar_velocity;
using rko_lio::core::diagonal_velocity_information;
using rko_lio::core::RadarIcpVelocityBlendResult;

}  // namespace

// Equal (isotropic) informations should produce a plain average of the two velocity estimates.
TEST(RadarContinuousFusion, IsotropicEqualInformationAverages)
{
  const Eigen::Vector3d icp_velocity(1.0, 0.0, 0.0);
  const Eigen::Vector3d radar_velocity(3.0, 0.0, 0.0);
  const Eigen::Matrix3d icp_information = Eigen::Matrix3d::Identity() * 10.0;
  const Eigen::Matrix3d radar_information = Eigen::Matrix3d::Identity() * 10.0;

  const RadarIcpVelocityBlendResult result =
    blend_icp_radar_velocity(icp_velocity, icp_information, radar_velocity, radar_information);

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.velocity.isApprox(Eigen::Vector3d(2.0, 0.0, 0.0), 1e-9));
}

// A much more confident sensor should dominate the fused estimate.
TEST(RadarContinuousFusion, HigherInformationDominates)
{
  const Eigen::Vector3d icp_velocity(0.0, 0.0, 0.0);
  const Eigen::Vector3d radar_velocity(5.0, 0.0, 0.0);
  // Radar information 1000x the ICP information along x: fused value should land very close
  // to the radar estimate.
  const Eigen::Matrix3d icp_information = Eigen::Matrix3d::Identity() * 1.0;
  const Eigen::Matrix3d radar_information = Eigen::Matrix3d::Identity() * 1000.0;

  const RadarIcpVelocityBlendResult result =
    blend_icp_radar_velocity(icp_velocity, icp_information, radar_velocity, radar_information);

  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.velocity.x(), 5.0, 0.02);
}

// Anisotropic case: radar is confident (high information) only along a lateral axis where ICP
// is weak (low information, e.g. a degenerate/near-singular direction), while ICP remains
// trusted along its own strong forward axis. The fused result should follow the radar along the
// weak axis but stay close to ICP along the strong axis -- this is the core behavior the
// continuous fusion feature depends on (see Config::radar_velocity_continuous_fusion).
TEST(RadarContinuousFusion, AnisotropicRadarDominatesOnlyWeakIcpAxis)
{
  const Eigen::Vector3d icp_velocity(2.0, 0.0, 0.0);    // ICP: confident forward motion
  const Eigen::Vector3d radar_velocity(2.0, 1.5, 0.0);  // radar agrees on x, sees lateral drift

  // ICP: strong information along x (forward), essentially zero along y (degenerate direction).
  Eigen::Matrix3d icp_information = Eigen::Matrix3d::Zero();
  icp_information(0, 0) = 1.0e4;
  icp_information(1, 1) = 1.0e-6;
  icp_information(2, 2) = 1.0e4;

  // Radar: weaker along x than ICP, but the only real signal along y.
  const Eigen::Matrix3d radar_information = diagonal_velocity_information(0.15, 0.5, 0.5);

  const RadarIcpVelocityBlendResult result =
    blend_icp_radar_velocity(icp_velocity, icp_information, radar_velocity, radar_information);

  ASSERT_TRUE(result.valid);
  // Forward axis: ICP's information swamps radar's, fused value stays close to ICP.
  EXPECT_NEAR(result.velocity.x(), icp_velocity.x(), 0.05);
  // Lateral axis: ICP contributes ~nothing, fused value should follow the radar's measurement.
  EXPECT_NEAR(result.velocity.y(), radar_velocity.y(), 0.05);
}

// Degenerate case: both informations are exactly singular along some axis (e.g. a completely
// unobserved direction for both sensors). The combined information is then singular too, so the
// blend must fail cleanly (valid == false) rather than returning a garbage/non-finite velocity.
TEST(RadarContinuousFusion, BothInformationsSingularFallsBackInvalid)
{
  const Eigen::Vector3d icp_velocity(1.0, 0.0, 0.0);
  const Eigen::Vector3d radar_velocity(1.0, 0.0, 0.0);

  Eigen::Matrix3d icp_information = Eigen::Matrix3d::Zero();
  icp_information(0, 0) = 10.0;
  icp_information(2, 2) = 10.0;
  // y axis left at 0: totally unobserved by ICP.

  Eigen::Matrix3d radar_information = Eigen::Matrix3d::Zero();
  radar_information(0, 0) = 5.0;
  radar_information(2, 2) = 5.0;
  // y axis also left at 0: totally unobserved by radar too (e.g. a 2D radar).

  const RadarIcpVelocityBlendResult result =
    blend_icp_radar_velocity(icp_velocity, icp_information, radar_velocity, radar_information);

  EXPECT_FALSE(result.valid);
}

// A single well-conditioned informative sensor plus a completely uninformative (zero) one should
// still solve fine: the combined information is exactly the informative sensor's, which is
// non-singular by construction.
TEST(RadarContinuousFusion, ZeroRadarInformationFallsBackToIcpOnly)
{
  const Eigen::Vector3d icp_velocity(4.0, -1.0, 0.2);
  const Eigen::Vector3d radar_velocity(0.0, 0.0, 0.0);  // irrelevant: zero information below

  const Eigen::Matrix3d icp_information = Eigen::Matrix3d::Identity() * 50.0;
  const Eigen::Matrix3d radar_information = Eigen::Matrix3d::Zero();

  const RadarIcpVelocityBlendResult result =
    blend_icp_radar_velocity(icp_velocity, icp_information, radar_velocity, radar_information);

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.velocity.isApprox(icp_velocity, 1e-9));
}

// Non-finite inputs must be rejected up front rather than propagating NaN/Inf.
TEST(RadarContinuousFusion, NonFiniteInputsAreInvalid)
{
  const Eigen::Vector3d icp_velocity(1.0, 0.0, 0.0);
  const Eigen::Vector3d bad_radar_velocity(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0);
  const Eigen::Matrix3d info = Eigen::Matrix3d::Identity();

  const RadarIcpVelocityBlendResult result =
    blend_icp_radar_velocity(icp_velocity, info, bad_radar_velocity, info);

  EXPECT_FALSE(result.valid);
}

TEST(RadarContinuousFusion, DiagonalVelocityInformationBuildsExpectedDiagonal)
{
  const Eigen::Matrix3d info = diagonal_velocity_information(0.1, 0.2, 0.5);
  EXPECT_NEAR(info(0, 0), 1.0 / (0.1 * 0.1), 1e-9);
  EXPECT_NEAR(info(1, 1), 1.0 / (0.2 * 0.2), 1e-9);
  EXPECT_NEAR(info(2, 2), 1.0 / (0.5 * 0.5), 1e-9);
  EXPECT_NEAR(info(0, 1), 0.0, 1e-12);
  EXPECT_NEAR(info(0, 2), 0.0, 1e-12);
  EXPECT_NEAR(info(1, 2), 0.0, 1e-12);
}
