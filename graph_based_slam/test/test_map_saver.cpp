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

// Characterization tests for the map-save decision logic extracted from
// saveGridDividedMap / saveSubmapToPCD. These pin the historical grid
// alignment, the truncated-corner cell filenames and the exact YAML /
// stdout formatting (fixed precision 1 metadata, fixed precision 10
// projector origin, default-float log lines) so the extraction and the
// later BackendCore reuse cannot silently change the Autoware map
// bundle.

#include <gtest/gtest.h>

#include <string>
#include <utility>

#include "graph_based_slam/map_saver.hpp"

namespace graphslam
{
namespace
{

using map_saver::CellFile;
using map_saver::GridBounds;
using map_saver::GridConfig;

GridConfig makeConfig(double grid_x = 20.0, double grid_y = 20.0)
{
  GridConfig config;
  config.grid_size_x = grid_x;
  config.grid_size_y = grid_y;
  return config;
}

TEST(MapSaverGridBounds, AlignsToGridAndCountsCells)
{
  const GridBounds bounds = map_saver::computeGridBounds(5.0, 7.0, 25.0, 33.0, makeConfig());
  EXPECT_DOUBLE_EQ(bounds.x_min, 0.0);
  EXPECT_DOUBLE_EQ(bounds.y_min, 0.0);
  EXPECT_EQ(bounds.nx, 2);
  EXPECT_EQ(bounds.ny, 2);
}

TEST(MapSaverGridBounds, NegativeCoordinatesFloorAwayFromZero)
{
  const GridBounds bounds = map_saver::computeGridBounds(-5.0, -25.0, 15.0, -1.0, makeConfig());
  EXPECT_DOUBLE_EQ(bounds.x_min, -20.0);
  EXPECT_DOUBLE_EQ(bounds.y_min, -40.0);
  EXPECT_EQ(bounds.nx, 2);
  EXPECT_EQ(bounds.ny, 2);
}

TEST(MapSaverGridBounds, DegenerateExtentClampsToOneCell)
{
  const GridBounds bounds = map_saver::computeGridBounds(0.0, 0.0, 0.0, 0.0, makeConfig());
  EXPECT_EQ(bounds.nx, 1);
  EXPECT_EQ(bounds.ny, 1);
}

TEST(MapSaverCellIndex, BoundaryPointBelongsToTheUpperCell)
{
  const GridConfig config = makeConfig();
  GridBounds bounds;
  bounds.x_min = 0.0;
  bounds.y_min = 0.0;
  EXPECT_EQ(map_saver::cellIndexFor(0.0, 0.0, bounds, config), std::make_pair(0, 0));
  EXPECT_EQ(map_saver::cellIndexFor(19.999, 19.999, bounds, config), std::make_pair(0, 0));
  EXPECT_EQ(map_saver::cellIndexFor(20.0, 20.0, bounds, config), std::make_pair(1, 1));
}

TEST(MapSaverCellIndex, RoundTripsThroughTheCellCorner)
{
  const GridConfig config = makeConfig();
  const GridBounds bounds = map_saver::computeGridBounds(-5.0, -25.0, 15.0, -1.0, config);
  const std::pair<int, int> key = map_saver::cellIndexFor(-3.5, -21.0, bounds, config);
  const CellFile cell = map_saver::makeCellFile(key, bounds, config);
  EXPECT_LE(cell.label_x, -3.5);
  EXPECT_GT(cell.label_x + config.grid_size_x, -3.5);
  EXPECT_LE(cell.label_y, -21.0);
  EXPECT_GT(cell.label_y + config.grid_size_y, -21.0);
}

TEST(MapSaverCellFile, NameIsTruncatedCornerPair)
{
  const GridConfig config = makeConfig();
  GridBounds bounds;
  bounds.x_min = -20.0;
  bounds.y_min = -40.0;
  const CellFile origin_cell = map_saver::makeCellFile(std::make_pair(0, 0), bounds, config);
  EXPECT_EQ(origin_cell.filename, "-20_-40.pcd");
  EXPECT_EQ(origin_cell.label_x, -20);
  EXPECT_EQ(origin_cell.label_y, -40);
  const CellFile shifted_cell = map_saver::makeCellFile(std::make_pair(1, 2), bounds, config);
  EXPECT_EQ(shifted_cell.filename, "0_0.pcd");
}

TEST(MapSaverMetadata, HeaderUsesFixedPrecisionOne)
{
  EXPECT_EQ(
    map_saver::metadataHeader(makeConfig()),
    "x_resolution: 20.0\ny_resolution: 20.0\n");
  EXPECT_EQ(
    map_saver::metadataHeader(makeConfig(12.5, 7.25)),
    "x_resolution: 12.5\ny_resolution: 7.2\n");
}

TEST(MapSaverMetadata, EntryListsFilenameAndCorner)
{
  CellFile cell;
  cell.filename = "0_20.pcd";
  cell.label_x = 0;
  cell.label_y = 20;
  EXPECT_EQ(map_saver::metadataEntry(cell), "0_20.pcd: [0, 20]\n");
}

TEST(MapSaverProjectorInfo, LocalCartesianWritesOriginAtPrecisionTen)
{
  EXPECT_EQ(
    map_saver::projectorInfoYaml(true, 35.5, 139.75),
    "projector_type: LocalCartesian\n"
    "vertical_datum: WGS84\n"
    "map_origin:\n"
    "  latitude: 35.5000000000\n"
    "  longitude: 139.7500000000\n");
}

TEST(MapSaverProjectorInfo, LocalOmitsOrigin)
{
  EXPECT_EQ(map_saver::projectorInfoYaml(false, 35.5, 139.75), "projector_type: Local\n");
}

TEST(MapSaverProjectorInfo, LogLineNamesTheProjectorType)
{
  EXPECT_EQ(
    map_saver::projectorInfoLogLine(true, "/tmp/map_projector_info.yaml"),
    "Saved Autoware map projector info (LocalCartesian): /tmp/map_projector_info.yaml");
  EXPECT_EQ(
    map_saver::projectorInfoLogLine(false, "/tmp/map_projector_info.yaml"),
    "Saved Autoware map projector info (Local): /tmp/map_projector_info.yaml");
}

TEST(MapSaverLogLines, DownsampleUsesDefaultFloatFormatting)
{
  EXPECT_EQ(
    map_saver::downsampleLogLine(1000, 200, 0.2),
    "Map points: 1000 -> 200 (leaf=0.2m)");
  EXPECT_EQ(
    map_saver::downsampleLogLine(50, 50, 1.0),
    "Map points: 50 -> 50 (leaf=1m)");
}

TEST(MapSaverLogLines, SavedMapUsesDefaultFloatFormatting)
{
  EXPECT_EQ(
    map_saver::savedMapLogLine(5, makeConfig(), "/tmp/out/pointcloud_map"),
    "Saved grid-divided map: 5 cells (20x20m) to /tmp/out/pointcloud_map");
  EXPECT_EQ(
    map_saver::savedMapLogLine(1, makeConfig(12.5, 12.5), "/tmp/out/pointcloud_map"),
    "Saved grid-divided map: 1 cells (12.5x12.5m) to /tmp/out/pointcloud_map");
}

TEST(MapSaverSubmapCache, PathAppendsIndexedFilename)
{
  EXPECT_EQ(map_saver::submapCachePath("/cache", 0), "/cache/submap_0.pcd");
  EXPECT_EQ(map_saver::submapCachePath("/cache", 42), "/cache/submap_42.pcd");
}

TEST(MapSaverBundle, TrajectoryUsesOfflineRunnerTumPrecision)
{
  map_saver::TrajectoryPose pose;
  pose.timestamp = 1.25;
  pose.tx = 2.0;
  pose.ty = -3.5;
  pose.tz = 0.125;
  pose.qz = 0.5;
  pose.qw = 0.866025403784;
  EXPECT_EQ(
    map_saver::trajectoryTumLine(pose),
    "1.250000000 2.000000000 -3.500000000 0.125000000 0.000000000 "
    "0.000000000 0.500000000 0.866025404");
}

TEST(MapSaverBundle, LoopEdgeMatchesOfflineCsvContract)
{
  map_saver::LoopEdgeRecord edge;
  edge.from = 6;
  edge.to = 604;
  edge.fitness = 0.47313574856385182;
  edge.tx = 1.5;
  edge.qw = 1.0;
  EXPECT_EQ(
    map_saver::loopEdgesCsvHeader(),
    "from,to,fitness,tx,ty,tz,qx,qy,qz,qw");
  EXPECT_EQ(
    map_saver::loopEdgeCsvLine(edge),
    "6,604,0.47313574856385182,1.5,0,0,0,0,0,1");
}

TEST(MapSaverBundle, ManifestNamesEveryRequiredArtifactDeterministically)
{
  map_saver::BundleManifest manifest;
  manifest.submap_count = 42;
  manifest.loop_edge_count = 3;
  manifest.map_leaf_size = 0.2;
  manifest.grid_size_x = 20.0;
  manifest.grid_size_y = 10.0;
  manifest.dynamic_object_filter = true;
  manifest.planar_map_filter = true;
  manifest.planar_map_filter_voxel_size = 0.1;
  manifest.planar_map_filter_min_neighbors = 3;
  manifest.planar_map_filter_max_small_eigenvalue_ratio = 0.24;
  manifest.planar_map_filter_min_middle_eigenvalue_ratio = 0.0;
  manifest.planar_map_filter_min_retained_ratio = 0.9;
  EXPECT_EQ(
    map_saver::bundleManifestYaml(manifest),
    "format_version: 1\n"
    "frame_id: map\n"
    "submap_count: 42\n"
    "loop_edge_count: 3\n"
    "map_leaf_size_m: 0.20000000000000001\n"
    "grid_size_x_m: 20\n"
    "grid_size_y_m: 10\n"
    "dynamic_object_filter: true\n"
    "planar_map_filter: true\n"
    "planar_map_filter_voxel_size_m: 0.10000000000000001\n"
    "planar_map_filter_min_neighbors: 3\n"
    "planar_map_filter_max_small_eigenvalue_ratio: 0.23999999999999999\n"
    "planar_map_filter_min_middle_eigenvalue_ratio: 0\n"
    "planar_map_filter_min_retained_ratio: 0.90000000000000002\n"
    "artifacts:\n"
    "  full_map: map.pcd\n"
    "  pointcloud_map: pointcloud_map\n"
    "  trajectory: trajectory_optimized.tum\n"
    "  pose_graph: pose_graph.g2o\n"
    "  loop_edges: loop_edges.csv\n"
    "  projector_info: map_projector_info.yaml\n");
}

}  // namespace
}  // namespace graphslam
