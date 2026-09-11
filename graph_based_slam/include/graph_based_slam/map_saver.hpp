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

#ifndef GRAPH_BASED_SLAM__MAP_SAVER_HPP_
#define GRAPH_BASED_SLAM__MAP_SAVER_HPP_

// Pure decision logic for the map-save path: grid bounds, the
// point-to-cell assignment, the Autoware metadata / projector-info YAML
// content and the operator log lines. The PCD file I/O, the directory
// management and the PCL downsampling stay in the ROS component; every
// byte that ends up in a YAML file or on stdout is produced here so the
// offline BackendCore can reuse it unchanged (docs/roadmap/v0.6.md,
// Phase 2).

#include <cmath>
#include <cstdio>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>

namespace graphslam
{
namespace map_saver
{

struct GridConfig
{
  double grid_size_x{20.0};
  double grid_size_y{20.0};
};

struct GridBounds
{
  double x_min{0.0};
  double y_min{0.0};
  int nx{1};
  int ny{1};
};

struct TrajectoryPose
{
  double timestamp{0.0};
  double tx{0.0};
  double ty{0.0};
  double tz{0.0};
  double qx{0.0};
  double qy{0.0};
  double qz{0.0};
  double qw{1.0};
};

struct LoopEdgeRecord
{
  int from{-1};
  int to{-1};
  double fitness{0.0};
  double tx{0.0};
  double ty{0.0};
  double tz{0.0};
  double qx{0.0};
  double qy{0.0};
  double qz{0.0};
  double qw{1.0};
};

struct BundleManifest
{
  std::string frame_id{"map"};
  std::size_t submap_count{0};
  std::size_t loop_edge_count{0};
  double map_leaf_size{0.2};
  double grid_size_x{20.0};
  double grid_size_y{20.0};
  bool dynamic_object_filter{false};
  bool planar_map_filter{false};
  double planar_map_filter_voxel_size{0.1};
  int planar_map_filter_min_neighbors{3};
  double planar_map_filter_max_small_eigenvalue_ratio{0.24};
  double planar_map_filter_min_middle_eigenvalue_ratio{0.0};
  double planar_map_filter_min_retained_ratio{0.90};
};

inline std::string trajectoryTumLine(const TrajectoryPose & pose)
{
  char line[256];
  std::snprintf(
    line, sizeof(line), "%.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f",
    pose.timestamp, pose.tx, pose.ty, pose.tz, pose.qx, pose.qy, pose.qz, pose.qw);
  return std::string(line);
}

inline std::string loopEdgesCsvHeader()
{
  return "from,to,fitness,tx,ty,tz,qx,qy,qz,qw";
}

inline std::string loopEdgeCsvLine(const LoopEdgeRecord & edge)
{
  char line[512];
  std::snprintf(
    line, sizeof(line), "%d,%d,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g",
    edge.from, edge.to, edge.fitness, edge.tx, edge.ty, edge.tz,
    edge.qx, edge.qy, edge.qz, edge.qw);
  return std::string(line);
}

inline std::string bundleManifestYaml(const BundleManifest & manifest)
{
  std::ostringstream out;
  out << "format_version: 1\n";
  out << "frame_id: " << manifest.frame_id << "\n";
  out << "submap_count: " << manifest.submap_count << "\n";
  out << "loop_edge_count: " << manifest.loop_edge_count << "\n";
  out << std::setprecision(17);
  out << "map_leaf_size_m: " << manifest.map_leaf_size << "\n";
  out << "grid_size_x_m: " << manifest.grid_size_x << "\n";
  out << "grid_size_y_m: " << manifest.grid_size_y << "\n";
  out << "dynamic_object_filter: " << (manifest.dynamic_object_filter ? "true" : "false") << "\n";
  out << "planar_map_filter: " << (manifest.planar_map_filter ? "true" : "false") << "\n";
  out << "planar_map_filter_voxel_size_m: " <<
    manifest.planar_map_filter_voxel_size << "\n";
  out << "planar_map_filter_min_neighbors: " <<
    manifest.planar_map_filter_min_neighbors << "\n";
  out << "planar_map_filter_max_small_eigenvalue_ratio: " <<
    manifest.planar_map_filter_max_small_eigenvalue_ratio << "\n";
  out << "planar_map_filter_min_middle_eigenvalue_ratio: " <<
    manifest.planar_map_filter_min_middle_eigenvalue_ratio << "\n";
  out << "planar_map_filter_min_retained_ratio: " <<
    manifest.planar_map_filter_min_retained_ratio << "\n";
  out << "artifacts:\n";
  out << "  full_map: map.pcd\n";
  out << "  pointcloud_map: pointcloud_map\n";
  out << "  trajectory: trajectory_optimized.tum\n";
  out << "  pose_graph: pose_graph.g2o\n";
  out << "  loop_edges: loop_edges.csv\n";
  out << "  projector_info: map_projector_info.yaml\n";
  return out.str();
}

// Grid-aligned bounding box of the map. nx/ny keep the historical
// at-least-one-cell clamp.
inline GridBounds computeGridBounds(
  double min_x, double min_y, double max_x, double max_y, const GridConfig & config)
{
  GridBounds bounds;
  bounds.x_min = std::floor(min_x / config.grid_size_x) * config.grid_size_x;
  bounds.y_min = std::floor(min_y / config.grid_size_y) * config.grid_size_y;
  const double x_max = std::ceil(max_x / config.grid_size_x) * config.grid_size_x;
  const double y_max = std::ceil(max_y / config.grid_size_y) * config.grid_size_y;
  bounds.nx = static_cast<int>((x_max - bounds.x_min) / config.grid_size_x);
  bounds.ny = static_cast<int>((y_max - bounds.y_min) / config.grid_size_y);
  if (bounds.nx <= 0) {bounds.nx = 1;}
  if (bounds.ny <= 0) {bounds.ny = 1;}
  return bounds;
}

inline std::pair<int, int> cellIndexFor(
  double x, double y, const GridBounds & bounds, const GridConfig & config)
{
  return std::make_pair(
    static_cast<int>(std::floor((x - bounds.x_min) / config.grid_size_x)),
    static_cast<int>(std::floor((y - bounds.y_min) / config.grid_size_y)));
}

struct CellFile
{
  std::string filename;
  int label_x{0};
  int label_y{0};
};

// Filename and metadata label for one grid cell; the labels are the
// truncated lower-left corner coordinates Autoware's
// pointcloud_map_loader expects.
inline CellFile makeCellFile(
  const std::pair<int, int> & key, const GridBounds & bounds, const GridConfig & config)
{
  const double cell_x = bounds.x_min + key.first * config.grid_size_x;
  const double cell_y = bounds.y_min + key.second * config.grid_size_y;
  CellFile cell;
  cell.label_x = static_cast<int>(cell_x);
  cell.label_y = static_cast<int>(cell_y);
  std::ostringstream name;
  name << cell.label_x << "_" << cell.label_y << ".pcd";
  cell.filename = name.str();
  return cell;
}

inline std::string metadataHeader(const GridConfig & config)
{
  std::ostringstream out;
  out << std::fixed;
  out << "x_resolution: " << std::setprecision(1) << config.grid_size_x << "\n";
  out << "y_resolution: " << std::setprecision(1) << config.grid_size_y << "\n";
  return out.str();
}

inline std::string metadataEntry(const CellFile & cell)
{
  std::ostringstream out;
  out << cell.filename << ": [" << cell.label_x << ", " << cell.label_y << "]" << "\n";
  return out.str();
}

inline std::string projectorInfoYaml(bool gnss_origin_set, double origin_lat, double origin_lon)
{
  std::ostringstream out;
  out << std::fixed << std::setprecision(10);
  if (gnss_origin_set) {
    out << "projector_type: LocalCartesian" << "\n";
    out << "vertical_datum: WGS84" << "\n";
    out << "map_origin:" << "\n";
    out << "  latitude: " << origin_lat << "\n";
    out << "  longitude: " << origin_lon << "\n";
  } else {
    out << "projector_type: Local" << "\n";
  }
  return out.str();
}

inline std::string projectorInfoLogLine(bool gnss_origin_set, const std::string & proj_file)
{
  std::ostringstream out;
  out << "Saved Autoware map projector info ("
      << (gnss_origin_set ? "LocalCartesian" : "Local") << "): " << proj_file;
  return out.str();
}

inline std::string downsampleLogLine(
  std::size_t input_points, std::size_t output_points, double leaf_size)
{
  std::ostringstream out;
  out << "Map points: " << input_points << " -> " << output_points
      << " (leaf=" << leaf_size << "m)";
  return out.str();
}

inline std::string savedMapLogLine(
  int saved_cells, const GridConfig & config, const std::string & out_dir)
{
  std::ostringstream out;
  out << "Saved grid-divided map: " << saved_cells << " cells ("
      << config.grid_size_x << "x" << config.grid_size_y << "m) to " << out_dir;
  return out.str();
}

inline std::string submapCachePath(const std::string & cache_dir, int idx)
{
  return cache_dir + "/submap_" + std::to_string(idx) + ".pcd";
}

}  // namespace map_saver
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__MAP_SAVER_HPP_
