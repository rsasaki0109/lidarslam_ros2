#ifndef SCANMATCHER__VOXEL_GRID_SAFETY_HPP_
#define SCANMATCHER__VOXEL_GRID_SAFETY_HPP_

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <string_view>

namespace graphslam::voxel_grid_safety
{

enum class Reason
{
  kSafe,
  kNullInput,
  kInvalidLeafSize,
  kEmptyCloud,
  kNoFinitePoints,
  kNonFiniteDenseCloud,
  kVoxelIndexOverflow,
  kVoxelLayoutOverflow,
};

struct Report
{
  Reason reason {Reason::kSafe};
  double requested_leaf_size {0.0};
  float effective_leaf_size {0.0F};
  std::size_t input_points {0};
  std::size_t finite_points {0};
  std::size_t non_finite_points {0};
  std::array<float, 3> min_xyz {0.0F, 0.0F, 0.0F};
  std::array<float, 3> max_xyz {0.0F, 0.0F, 0.0F};
  std::array<std::int64_t, 3> min_voxel {0, 0, 0};
  std::array<std::int64_t, 3> max_voxel {0, 0, 0};
  std::array<std::uint64_t, 3> divisions {0, 0, 0};
  std::uint64_t layout_cells {0};

  bool safe() const
  {
    return reason == Reason::kSafe;
  }
};

inline constexpr std::uint64_t kMaxPclVoxelLayoutCells =
  static_cast<std::uint64_t>(std::numeric_limits<std::int32_t>::max());

inline const char * reasonCode(Reason reason)
{
  switch (reason) {
    case Reason::kSafe:
      return "VOXEL_GRID_SAFE";
    case Reason::kNullInput:
      return "VOXEL_GRID_NULL_INPUT";
    case Reason::kInvalidLeafSize:
      return "VOXEL_GRID_INVALID_LEAF_SIZE";
    case Reason::kEmptyCloud:
      return "VOXEL_GRID_EMPTY_CLOUD";
    case Reason::kNoFinitePoints:
      return "VOXEL_GRID_NO_FINITE_POINTS";
    case Reason::kNonFiniteDenseCloud:
      return "VOXEL_GRID_NONFINITE_DENSE_CLOUD";
    case Reason::kVoxelIndexOverflow:
      return "VOXEL_GRID_INDEX_OVERFLOW";
    case Reason::kVoxelLayoutOverflow:
      return "VOXEL_GRID_LAYOUT_OVERFLOW";
  }
  return "VOXEL_GRID_UNKNOWN";
}

inline Report inspectBounds(
  const std::array<float, 3> & min_xyz,
  const std::array<float, 3> & max_xyz,
  double requested_leaf_size,
  std::size_t input_points,
  std::size_t finite_points,
  std::size_t non_finite_points,
  bool is_dense)
{
  Report report;
  report.requested_leaf_size = requested_leaf_size;
  report.effective_leaf_size = static_cast<float>(requested_leaf_size);
  report.input_points = input_points;
  report.finite_points = finite_points;
  report.non_finite_points = non_finite_points;
  report.min_xyz = min_xyz;
  report.max_xyz = max_xyz;

  const float leaf = report.effective_leaf_size;
  if (
    !std::isfinite(requested_leaf_size) || requested_leaf_size <= 0.0 ||
    !std::isfinite(leaf) || leaf <= 0.0F)
  {
    report.reason = Reason::kInvalidLeafSize;
    return report;
  }
  const float inverse_leaf = 1.0F / leaf;
  if (!std::isfinite(inverse_leaf) || inverse_leaf <= 0.0F) {
    report.reason = Reason::kInvalidLeafSize;
    return report;
  }
  if (input_points == 0) {
    report.reason = Reason::kEmptyCloud;
    return report;
  }
  if (is_dense && non_finite_points > 0) {
    report.reason = Reason::kNonFiniteDenseCloud;
    return report;
  }
  if (finite_points == 0) {
    report.reason = Reason::kNoFinitePoints;
    return report;
  }

  constexpr double kMinIndex =
    static_cast<double>(std::numeric_limits<std::int32_t>::min());
  constexpr double kMaxIndex =
    static_cast<double>(std::numeric_limits<std::int32_t>::max());
  for (std::size_t axis = 0; axis < 3; ++axis) {
    // Use the same effective float leaf and float multiplication as PCL.
    // This avoids accepting a boundary that PCL would round outside int32.
    const float scaled_min = min_xyz[axis] * inverse_leaf;
    const float scaled_max = max_xyz[axis] * inverse_leaf;
    if (!std::isfinite(scaled_min) || !std::isfinite(scaled_max)) {
      report.reason = Reason::kVoxelIndexOverflow;
      return report;
    }
    const double floored_min = std::floor(static_cast<double>(scaled_min));
    const double floored_max = std::floor(static_cast<double>(scaled_max));
    if (
      floored_min < kMinIndex || floored_min > kMaxIndex ||
      floored_max < kMinIndex || floored_max > kMaxIndex)
    {
      report.reason = Reason::kVoxelIndexOverflow;
      return report;
    }
    report.min_voxel[axis] = static_cast<std::int64_t>(floored_min);
    report.max_voxel[axis] = static_cast<std::int64_t>(floored_max);
    report.divisions[axis] = static_cast<std::uint64_t>(
      report.max_voxel[axis] - report.min_voxel[axis]) + 1U;
  }

  std::uint64_t cells = 1;
  for (const std::uint64_t division : report.divisions) {
    if (division > kMaxPclVoxelLayoutCells / cells) {
      report.layout_cells = kMaxPclVoxelLayoutCells + 1U;
      report.reason = Reason::kVoxelLayoutOverflow;
      return report;
    }
    cells *= division;
  }
  report.layout_cells = cells;
  report.reason = Reason::kSafe;
  return report;
}

template<typename PointT>
Report inspect(
  const pcl::PointCloud<PointT> & input,
  double requested_leaf_size)
{
  std::array<float, 3> min_xyz {
    std::numeric_limits<float>::max(),
    std::numeric_limits<float>::max(),
    std::numeric_limits<float>::max()};
  std::array<float, 3> max_xyz {
    std::numeric_limits<float>::lowest(),
    std::numeric_limits<float>::lowest(),
    std::numeric_limits<float>::lowest()};
  std::size_t finite_points = 0;
  std::size_t non_finite_points = 0;

  for (const PointT & point : input.points) {
    const bool finite =
      std::isfinite(static_cast<double>(point.x)) &&
      std::isfinite(static_cast<double>(point.y)) &&
      std::isfinite(static_cast<double>(point.z));
    if (!finite) {
      ++non_finite_points;
      continue;
    }
    min_xyz[0] = std::min(min_xyz[0], point.x);
    min_xyz[1] = std::min(min_xyz[1], point.y);
    min_xyz[2] = std::min(min_xyz[2], point.z);
    max_xyz[0] = std::max(max_xyz[0], point.x);
    max_xyz[1] = std::max(max_xyz[1], point.y);
    max_xyz[2] = std::max(max_xyz[2], point.z);
    ++finite_points;
  }

  if (finite_points == 0) {
    min_xyz = {0.0F, 0.0F, 0.0F};
    max_xyz = {0.0F, 0.0F, 0.0F};
  }
  return inspectBounds(
    min_xyz, max_xyz, requested_leaf_size, input.size(), finite_points,
    non_finite_points, input.is_dense);
}

template<typename PointT>
Report filter(
  typename pcl::PointCloud<PointT>::ConstPtr input,
  double requested_leaf_size,
  pcl::PointCloud<PointT> & output)
{
  if (!input) {
    output = pcl::PointCloud<PointT>();
    Report report;
    report.reason = Reason::kNullInput;
    report.requested_leaf_size = requested_leaf_size;
    report.effective_leaf_size = static_cast<float>(requested_leaf_size);
    return report;
  }

  Report report = inspect(*input, requested_leaf_size);
  if (!report.safe()) {
    output = pcl::PointCloud<PointT>();
    return report;
  }

  pcl::VoxelGrid<PointT> voxel_grid;
  voxel_grid.setLeafSize(
    report.effective_leaf_size,
    report.effective_leaf_size,
    report.effective_leaf_size);
  voxel_grid.setInputCloud(input);
  voxel_grid.filter(output);
  return report;
}

inline std::string formatRejection(
  const Report & report,
  std::string_view stage,
  std::string_view parameter_name)
{
  std::ostringstream stream;
  stream << '[' << reasonCode(report.reason) << "] " << stage
         << " rejected before PCL VoxelGrid: " << parameter_name << '='
         << std::setprecision(9) << report.requested_leaf_size
         << " m (effective=" << report.effective_leaf_size << " m), points="
         << report.input_points << ", finite=" << report.finite_points
         << ", nonfinite=" << report.non_finite_points;

  if (report.finite_points > 0) {
    stream << ", min=[" << report.min_xyz[0] << ',' << report.min_xyz[1]
           << ',' << report.min_xyz[2] << "], max=[" << report.max_xyz[0]
           << ',' << report.max_xyz[1] << ',' << report.max_xyz[2] << ']';
  }
  const bool missing_usable_input =
    report.reason == Reason::kEmptyCloud ||
    report.reason == Reason::kNoFinitePoints ||
    report.reason == Reason::kNullInput;
  if (report.reason == Reason::kVoxelIndexOverflow) {
    stream << ". Check coordinate units, origin, and outliers; increase "
           << parameter_name << " only after confirming the intended map resolution";
  } else if (report.reason == Reason::kVoxelLayoutOverflow) {
    stream << ", divisions=[" << report.divisions[0] << ','
           << report.divisions[1] << ',' << report.divisions[2]
           << "], cells>" << kMaxPclVoxelLayoutCells
           << ". Check coordinate units/outliers or increase " << parameter_name
           << " after confirming the intended resolution";
  } else if (report.reason == Reason::kInvalidLeafSize) {
    stream << ". Set " << parameter_name << " to a finite value greater than zero";
  } else if (report.reason == Reason::kNonFiniteDenseCloud) {
    stream << ". Remove non-finite XYZ values or publish the cloud with is_dense=false";
  } else if (missing_usable_input) {
    stream << ". Check the input topic, field conversion, and range filter";
  }
  stream << "; the node remains active";
  return stream.str();
}

}  // namespace graphslam::voxel_grid_safety

#endif  // SCANMATCHER__VOXEL_GRID_SAFETY_HPP_
