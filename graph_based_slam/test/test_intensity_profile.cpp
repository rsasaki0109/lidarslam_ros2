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

#include "rko_lio/core/intensity_profile.hpp"

namespace
{

using rko_lio::core::build_intensity_profile;
using rko_lio::core::estimate_profile_shift;
using rko_lio::core::intensity_implied_velocity_along_axis;
using rko_lio::core::IntensityProfile;
using rko_lio::core::IntensityProfileConfig;
using rko_lio::core::ProfileShiftResult;
using rko_lio::core::unit_axis_from_step;

// A handful of Gaussian bumps along the axis, at distinctive (non-periodic)
// positions -- stands in for reflectivity texture (lights/signs/cables) that
// varies along a self-similar tunnel and gives cross-correlation an
// unambiguous peak.
double texture(const double s)
{
  auto bump = [s](const double center, const double amplitude, const double width) {
      const double d = (s - center) / width;
      return amplitude * std::exp(-0.5 * d * d);
    };
  return bump(-18.0, 3.0, 1.2) + bump(-7.0, 1.5, 0.8) + bump(3.5, 2.5, 1.5) + bump(14.0, 2.0, 1.0) +
         bump(21.0, 1.0, 0.6);
}

// Dense samples of `texture` along the x axis so every 0.25 m bin fills.
// `position_offset` shifts where each sample is *placed* while the intensity
// value still comes from `texture` evaluated at the *unshifted* coordinate,
// i.e. the built profile satisfies profile(p) = texture(p - position_offset).
// Per estimate_profile_shift's documented convention (profile_a[i] ~=
// profile_b[i - shift_bins]), correlating a position_offset=0 profile against
// a position_offset=d profile recovers shift_m = -d.
std::pair<std::vector<Eigen::Vector3d>, std::vector<float>> makeTexturePoints(
  const double position_offset, const double step = 0.05, const double range = 25.0)
{
  std::vector<Eigen::Vector3d> points;
  std::vector<float> intensities;
  for (double s = -range; s <= range; s += step) {
    points.emplace_back(s + position_offset, 0.0, 0.0);
    intensities.push_back(static_cast<float>(texture(s)));
  }
  return {points, intensities};
}

}  // namespace

TEST(IntensityProfile, RecoversKnownShiftWithinTolerance)
{
  const auto [points_a, intensities_a] = makeTexturePoints(0.0);
  const auto [points_b, intensities_b] = makeTexturePoints(0.6);

  IntensityProfileConfig config;
  config.bin_size_m = 0.25;
  config.half_length_m = 30.0;
  config.max_shift_m = 1.5;
  config.min_correlation = 0.6;
  config.min_filled_bins = 40;

  const Eigen::Vector3d axis(1.0, 0.0, 0.0);
  const Eigen::Vector3d origin = Eigen::Vector3d::Zero();
  const IntensityProfile profile_a = build_intensity_profile(points_a, intensities_a, axis, origin,
    config);
  const IntensityProfile profile_b = build_intensity_profile(points_b, intensities_b, axis, origin,
    config);
  ASSERT_TRUE(profile_a.valid);
  ASSERT_TRUE(profile_b.valid);

  const ProfileShiftResult result = estimate_profile_shift(profile_a, profile_b, config);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.shift_m, -0.6, 0.05);
  EXPECT_GT(result.correlation, 0.9);
}

TEST(IntensityProfile, RecoversPositiveShift)
{
  const auto [points_a, intensities_a] = makeTexturePoints(0.0);
  const auto [points_b, intensities_b] = makeTexturePoints(-0.9);

  IntensityProfileConfig config;
  config.bin_size_m = 0.25;
  config.max_shift_m = 1.5;
  config.min_correlation = 0.6;
  config.min_filled_bins = 40;

  const Eigen::Vector3d axis(1.0, 0.0, 0.0);
  const Eigen::Vector3d origin = Eigen::Vector3d::Zero();
  const IntensityProfile profile_a = build_intensity_profile(points_a, intensities_a, axis, origin,
    config);
  const IntensityProfile profile_b = build_intensity_profile(points_b, intensities_b, axis, origin,
    config);
  ASSERT_TRUE(profile_a.valid);
  ASSERT_TRUE(profile_b.valid);

  const ProfileShiftResult result = estimate_profile_shift(profile_a, profile_b, config);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.shift_m, 0.9, 0.05);
}

TEST(IntensityProfile, LowCorrelationIsRejected)
{
  IntensityProfileConfig config;
  config.bin_size_m = 0.25;
  config.max_shift_m = 1.5;
  config.min_correlation = 0.6;
  config.min_filled_bins = 40;

  const Eigen::Vector3d axis(1.0, 0.0, 0.0);
  const Eigen::Vector3d origin = Eigen::Vector3d::Zero();

  std::mt19937 rng(123);
  std::uniform_real_distribution<float> noise(0.0F, 1.0F);
  std::vector<Eigen::Vector3d> points_a;
  std::vector<float> intensities_a;
  std::vector<Eigen::Vector3d> points_b;
  std::vector<float> intensities_b;
  for (double s = -25.0; s <= 25.0; s += 0.05) {
    points_a.emplace_back(s, 0.0, 0.0);
    intensities_a.push_back(noise(rng));
    points_b.emplace_back(s, 0.0, 0.0);
    intensities_b.push_back(noise(rng));  // independent noise: uncorrelated with a
  }

  const IntensityProfile profile_a = build_intensity_profile(points_a, intensities_a, axis, origin,
    config);
  const IntensityProfile profile_b = build_intensity_profile(points_b, intensities_b, axis, origin,
    config);
  ASSERT_TRUE(profile_a.valid);
  ASSERT_TRUE(profile_b.valid);

  const ProfileShiftResult result = estimate_profile_shift(profile_a, profile_b, config);
  EXPECT_FALSE(result.valid);
}

TEST(IntensityProfile, UnderfilledScanIsRejected)
{
  IntensityProfileConfig config;
  config.bin_size_m = 0.25;
  config.min_filled_bins = 40;

  const Eigen::Vector3d axis(1.0, 0.0, 0.0);
  const Eigen::Vector3d origin = Eigen::Vector3d::Zero();

  // Only a handful of points -- far fewer than min_filled_bins distinct bins.
  std::vector<Eigen::Vector3d> points = {
    {-1.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}};
  std::vector<float> intensities = {10.0F, 20.0F, 30.0F, 40.0F};

  const IntensityProfile profile = build_intensity_profile(points, intensities, axis, origin,
    config);
  EXPECT_FALSE(profile.valid);
  EXPECT_LT(profile.filled_count, config.min_filled_bins);
}

TEST(IntensityProfile, EmptyInputIsRejected)
{
  IntensityProfileConfig config;
  const Eigen::Vector3d axis(1.0, 0.0, 0.0);
  const Eigen::Vector3d origin = Eigen::Vector3d::Zero();
  const IntensityProfile profile = build_intensity_profile({}, {}, axis, origin, config);
  EXPECT_FALSE(profile.valid);

  const ProfileShiftResult result = estimate_profile_shift(profile, profile, config);
  EXPECT_FALSE(result.valid);
}

TEST(IntensityProfile, MismatchedSizesIsRejected)
{
  IntensityProfileConfig config;
  const Eigen::Vector3d axis(1.0, 0.0, 0.0);
  const Eigen::Vector3d origin = Eigen::Vector3d::Zero();
  const std::vector<Eigen::Vector3d> points = {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};
  const std::vector<float> intensities = {1.0F};
  const IntensityProfile profile = build_intensity_profile(points, intensities, axis, origin,
    config);
  EXPECT_FALSE(profile.valid);
}

// --- unit_axis_from_step / intensity_implied_velocity_along_axis -----------------------------
// Pure-function coverage for the intensity velocity-disagreement gate's axis selection and
// shift-to-velocity algebra (see core/lio.cpp's intensity_disagreement_gate block).

TEST(UnitAxisFromStep, NormalizesALongEnoughStep)
{
  const Eigen::Vector3d step(2.0, 0.0, 0.0);
  const auto axis = unit_axis_from_step(step, 0.01);
  ASSERT_TRUE(axis.has_value());
  EXPECT_NEAR(axis->norm(), 1.0, 1.0e-9);
  EXPECT_NEAR((*axis)(0), 1.0, 1.0e-9);
}

TEST(UnitAxisFromStep, RejectsStepShorterThanMinimum)
{
  const Eigen::Vector3d step(0.005, 0.0, 0.0);
  EXPECT_FALSE(unit_axis_from_step(step, 0.01).has_value());
}

TEST(UnitAxisFromStep, RejectsNonPositiveMinimum)
{
  const Eigen::Vector3d step(2.0, 0.0, 0.0);
  EXPECT_FALSE(unit_axis_from_step(step, 0.0).has_value());
  EXPECT_FALSE(unit_axis_from_step(step, -1.0).has_value());
}

TEST(IntensityImpliedVelocity, ZeroShiftMatchesInitialGuessDisplacement)
{
  // No correlation shift: the initial guess was already correct, so the implied velocity is
  // just the initial-guess displacement over dt, exactly like a plain finite difference.
  const Eigen::Vector3d axis(1.0, 0.0, 0.0);
  const Eigen::Vector3d initial_guess_translation(5.0, 0.0, 0.0);
  const Eigen::Vector3d previous_origin(0.0, 0.0, 0.0);
  const double v = intensity_implied_velocity_along_axis(axis, initial_guess_translation,
    previous_origin, 0.0, 2.0);
  EXPECT_NEAR(v, 2.5, 1.0e-9);  // (5 - 0) / 2
}

TEST(IntensityImpliedVelocity, ShiftAddsToTheImpliedDisplacement)
{
  // A positive shift_m means the texture is further along +axis than the initial guess placed
  // it, i.e. the true displacement -- and hence the implied velocity -- is larger.
  const Eigen::Vector3d axis(1.0, 0.0, 0.0);
  const Eigen::Vector3d initial_guess_translation(5.0, 0.0, 0.0);
  const Eigen::Vector3d previous_origin(0.0, 0.0, 0.0);
  const double v = intensity_implied_velocity_along_axis(axis, initial_guess_translation,
    previous_origin, 0.5, 2.0);
  EXPECT_NEAR(v, 2.75, 1.0e-9);  // (5 + 0.5 - 0) / 2
}

TEST(IntensityImpliedVelocity, OnlyTheAxisProjectionMatters)
{
  // A non-axis-aligned translation/origin should collapse to the same result as the aligned
  // case above, since only the dot product with `axis` matters.
  const Eigen::Vector3d axis(1.0, 0.0, 0.0);
  const Eigen::Vector3d initial_guess_translation(5.0, 3.0, -7.0);
  const Eigen::Vector3d previous_origin(0.0, 9.0, 4.0);
  const double v = intensity_implied_velocity_along_axis(axis, initial_guess_translation,
    previous_origin, 0.0, 2.0);
  EXPECT_NEAR(v, 2.5, 1.0e-9);
}

TEST(IntensityImpliedVelocity, NonPositiveDtReturnsZero)
{
  const Eigen::Vector3d axis(1.0, 0.0, 0.0);
  const Eigen::Vector3d initial_guess_translation(5.0, 0.0, 0.0);
  const Eigen::Vector3d previous_origin(0.0, 0.0, 0.0);
  EXPECT_EQ(intensity_implied_velocity_along_axis(axis, initial_guess_translation, previous_origin,
    0.5, 0.0), 0.0);
  EXPECT_EQ(intensity_implied_velocity_along_axis(axis, initial_guess_translation, previous_origin,
    0.5, -1.0), 0.0);
}
