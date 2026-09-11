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
//    in the documentation and/or other materials provided with the
//    distribution.
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

#include "filesystem_io_ports.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <utility>
#include <vector>

#include <pcl/io/pcd_io.h>  // NOLINT(build/include_order)

#include "graph_based_slam/map_saver.hpp"

namespace graphslam
{
namespace adapters
{
namespace
{
void createParentDirectory(const std::string & path)
{
  const std::filesystem::path parent = std::filesystem::path(path).parent_path();
  if (!parent.empty()) {
    std::filesystem::create_directories(parent);
  }
}
}  // namespace

PcdSubmapStorage::PcdSubmapStorage(std::string directory)
: directory_(std::move(directory))
{
  std::filesystem::create_directories(directory_);
}

bool PcdSubmapStorage::store(int index, const ports::PointCloud & cloud)
{
  if (index < 0) {return false;}
  std::lock_guard<std::mutex> lock(mutex_);
  return pcl::io::savePCDFileBinaryCompressed(
    map_saver::submapCachePath(directory_, index), cloud) == 0;
}

std::vector<bool> PcdSubmapStorage::storeBatch(
  const std::vector<ports::SubmapWrite> & writes)
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<bool> results;
  results.reserve(writes.size());
  for (const auto & write : writes) {
    const bool stored = write.index >= 0 && write.cloud &&
      pcl::io::savePCDFileBinaryCompressed(
      map_saver::submapCachePath(directory_, write.index), *write.cloud) == 0;
    results.push_back(stored);
  }
  return results;
}

ports::PointCloudPtr PcdSubmapStorage::load(int index) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  ports::PointCloudPtr cloud(new ports::PointCloud);
  if (index < 0 ||
    pcl::io::loadPCDFile(map_saver::submapCachePath(directory_, index), *cloud) == -1)
  {
    cloud->clear();
  }
  return cloud;
}

bool FilesystemMapOutput::prepareDirectory(const std::string & directory)
{
  std::error_code error;
  std::filesystem::create_directories(directory, error);
  return !error;
}

bool FilesystemMapOutput::removeByExtension(
  const std::string & directory, const std::vector<std::string> & extensions)
{
  if (!std::filesystem::exists(directory)) {return true;}
  std::error_code error;
  for (const auto & entry : std::filesystem::directory_iterator(directory)) {
    const std::string extension = entry.path().extension().string();
    if (std::find(extensions.begin(), extensions.end(), extension) != extensions.end()) {
      std::filesystem::remove(entry.path(), error);
      if (error) {return false;}
    }
  }
  return true;
}

bool FilesystemMapOutput::writeBytes(
  const std::string & artifact, const std::string & bytes)
{
  createParentDirectory(artifact);
  std::ofstream output(artifact, std::ios::binary);
  if (!output.is_open()) {return false;}
  output.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
  return output.good();
}

bool FilesystemMapOutput::appendBytes(
  const std::string & artifact, const std::string & bytes)
{
  createParentDirectory(artifact);
  std::ofstream output(artifact, std::ios::binary | std::ios::app);
  if (!output.is_open()) {return false;}
  output.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
  return output.good();
}

bool FilesystemMapOutput::writePointCloud(
  const std::string & artifact, const ports::PointCloud & cloud)
{
  createParentDirectory(artifact);
  return pcl::io::savePCDFileBinaryCompressed(artifact, cloud) == 0;
}

bool FilesystemMapOutput::writePointCloudXyz(
  const std::string & artifact, const ports::PointCloudXyz & cloud)
{
  createParentDirectory(artifact);
  return pcl::io::savePCDFileBinary(artifact, cloud) == 0;
}

}  // namespace adapters
}  // namespace graphslam
