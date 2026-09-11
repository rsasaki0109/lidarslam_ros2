#include <gtest/gtest.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>

#include <cmath>
#include <limits>
#include <string>

#include "scanmatcher/voxel_grid_safety.hpp"

namespace
{

using graphslam::voxel_grid_safety::Reason;
using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;

Point makePoint(float x, float y, float z, float intensity = 0.0F)
{
  Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  point.intensity = intensity;
  return point;
}

TEST(VoxelGridSafety, ValidCloudKeepsPclFilteringUnchanged)
{
  Cloud::Ptr input(new Cloud());
  input->push_back(makePoint(0.01F, 0.02F, 0.03F, 1.0F));
  input->push_back(makePoint(0.04F, 0.02F, 0.03F, 3.0F));
  input->push_back(makePoint(0.15F, 0.02F, 0.03F, 5.0F));

  Cloud expected;
  pcl::VoxelGrid<Point> direct_filter;
  direct_filter.setLeafSize(0.1F, 0.1F, 0.1F);
  direct_filter.setInputCloud(input);
  direct_filter.filter(expected);

  Cloud actual;
  const auto report = graphslam::voxel_grid_safety::filter<Point>(
    input, 0.1, actual);

  ASSERT_TRUE(report.safe());
  ASSERT_EQ(expected.size(), actual.size());
  for (std::size_t index = 0; index < expected.size(); ++index) {
    EXPECT_FLOAT_EQ(expected[index].x, actual[index].x);
    EXPECT_FLOAT_EQ(expected[index].y, actual[index].y);
    EXPECT_FLOAT_EQ(expected[index].z, actual[index].z);
    EXPECT_FLOAT_EQ(expected[index].intensity, actual[index].intensity);
  }
}

TEST(VoxelGridSafety, RejectsIssue69LayoutInsteadOfReturningUnfilteredCloud)
{
  Cloud::Ptr input(new Cloud());
  input->push_back(makePoint(-200.0F, -200.0F, -10.0F));
  input->push_back(makePoint(200.0F, 200.0F, 10.0F));
  Cloud output;
  output.push_back(makePoint(1.0F, 2.0F, 3.0F));

  const auto report = graphslam::voxel_grid_safety::filter<Point>(
    input, 0.1, output);

  EXPECT_EQ(Reason::kVoxelLayoutOverflow, report.reason);
  EXPECT_EQ((std::array<std::uint64_t, 3>{4001, 4001, 201}), report.divisions);
  EXPECT_TRUE(output.empty());
  EXPECT_EQ(
    std::string("VOXEL_GRID_LAYOUT_OVERFLOW"),
    graphslam::voxel_grid_safety::reasonCode(report.reason));
}

TEST(VoxelGridSafety, AcceptsLastCubicLayoutBelowPclInt32Limit)
{
  const auto report = graphslam::voxel_grid_safety::inspectBounds(
    {0.0F, 0.0F, 0.0F}, {1289.0F, 1289.0F, 1289.0F}, 1.0,
    2, 2, 0, true);

  EXPECT_TRUE(report.safe());
  EXPECT_EQ(2146689000U, report.layout_cells);
}

TEST(VoxelGridSafety, RejectsFirstCubicLayoutAbovePclInt32Limit)
{
  const auto report = graphslam::voxel_grid_safety::inspectBounds(
    {0.0F, 0.0F, 0.0F}, {1290.0F, 1290.0F, 1290.0F}, 1.0,
    2, 2, 0, true);

  EXPECT_EQ(Reason::kVoxelLayoutOverflow, report.reason);
  EXPECT_EQ((std::array<std::uint64_t, 3>{1291, 1291, 1291}), report.divisions);
}

TEST(VoxelGridSafety, HandlesNegativeVoxelIndices)
{
  const auto report = graphslam::voxel_grid_safety::inspectBounds(
    {-0.2F, -0.1F, -1.0F}, {0.2F, 0.1F, 1.0F}, 0.1,
    2, 2, 0, true);

  EXPECT_TRUE(report.safe());
  EXPECT_EQ(-2, report.min_voxel[0]);
  EXPECT_EQ(2, report.max_voxel[0]);
}

TEST(VoxelGridSafety, RejectsAbsoluteVoxelIndexOverflow)
{
  const float outside_int32 = std::ldexp(1.0F, 31);
  const auto report = graphslam::voxel_grid_safety::inspectBounds(
    {outside_int32, 0.0F, 0.0F}, {outside_int32, 0.0F, 0.0F}, 1.0,
    1, 1, 0, true);

  EXPECT_EQ(Reason::kVoxelIndexOverflow, report.reason);
}

TEST(VoxelGridSafety, RejectsInvalidEffectiveLeafSizes)
{
  constexpr std::array<float, 3> min_xyz {0.0F, 0.0F, 0.0F};
  constexpr std::array<float, 3> max_xyz {1.0F, 1.0F, 1.0F};
  for (const double leaf : {
      0.0,
      -0.1,
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity(),
      1.0e-50})
  {
    const auto report = graphslam::voxel_grid_safety::inspectBounds(
      min_xyz, max_xyz, leaf, 2, 2, 0, true);
    EXPECT_EQ(Reason::kInvalidLeafSize, report.reason);
  }
}

TEST(VoxelGridSafety, RejectsDenseCloudThatContainsNonFiniteXyz)
{
  Cloud cloud;
  cloud.is_dense = true;
  cloud.push_back(makePoint(0.0F, 0.0F, 0.0F));
  cloud.push_back(makePoint(
    std::numeric_limits<float>::quiet_NaN(), 1.0F, 1.0F));

  const auto report = graphslam::voxel_grid_safety::inspect(cloud, 0.1);

  EXPECT_EQ(Reason::kNonFiniteDenseCloud, report.reason);
  EXPECT_EQ(1U, report.finite_points);
  EXPECT_EQ(1U, report.non_finite_points);
}

TEST(VoxelGridSafety, AllowsNonDenseCloudAndPclDropsNonFiniteXyz)
{
  Cloud::Ptr input(new Cloud());
  input->is_dense = false;
  input->push_back(makePoint(0.0F, 0.0F, 0.0F));
  input->push_back(makePoint(
    std::numeric_limits<float>::quiet_NaN(), 1.0F, 1.0F));
  Cloud output;

  const auto report = graphslam::voxel_grid_safety::filter<Point>(
    input, 0.1, output);

  EXPECT_TRUE(report.safe());
  ASSERT_EQ(1U, output.size());
  EXPECT_TRUE(std::isfinite(output.front().x));
}

TEST(VoxelGridSafety, RejectsEmptyNullAndAllNonFiniteInputs)
{
  Cloud empty;
  EXPECT_EQ(
    Reason::kEmptyCloud,
    graphslam::voxel_grid_safety::inspect(empty, 0.1).reason);

  Cloud::ConstPtr null_input;
  Cloud output;
  EXPECT_EQ(
    Reason::kNullInput,
    graphslam::voxel_grid_safety::filter<Point>(null_input, 0.1, output).reason);

  Cloud all_non_finite;
  all_non_finite.is_dense = false;
  all_non_finite.push_back(makePoint(
    std::numeric_limits<float>::infinity(), 0.0F, 0.0F));
  EXPECT_EQ(
    Reason::kNoFinitePoints,
    graphslam::voxel_grid_safety::inspect(all_non_finite, 0.1).reason);
}

TEST(VoxelGridSafety, DiagnosticNamesReasonParameterAndRecovery)
{
  const auto report = graphslam::voxel_grid_safety::inspectBounds(
    {-200.0F, -200.0F, -10.0F}, {200.0F, 200.0F, 10.0F}, 0.1,
    2, 2, 0, true);
  const std::string message = graphslam::voxel_grid_safety::formatRejection(
    report, "map_update", "vg_size_for_map");

  EXPECT_NE(std::string::npos, message.find("[VOXEL_GRID_LAYOUT_OVERFLOW]"));
  EXPECT_NE(std::string::npos, message.find("map_update rejected before PCL"));
  EXPECT_NE(std::string::npos, message.find("vg_size_for_map"));
  EXPECT_NE(std::string::npos, message.find("coordinate units/outliers"));
  EXPECT_NE(std::string::npos, message.find("the node remains active"));
}

}  // namespace
