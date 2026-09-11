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

#ifndef GRAPH_BASED_SLAM__EXTERNAL_IO_PORTS_HPP_
#define GRAPH_BASED_SLAM__EXTERNAL_IO_PORTS_HPP_

#include <algorithm>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <pcl/point_cloud.h>  // NOLINT(build/include_order)
#include <pcl/point_types.h>  // NOLINT(build/include_order)

namespace graphslam
{
namespace ports
{

using PointCloud = pcl::PointCloud<pcl::PointXYZI>;
using PointCloudPtr = PointCloud::Ptr;
using PointCloudXyz = pcl::PointCloud<pcl::PointXYZ>;

struct SubmapWrite
{
  int index {-1};
  PointCloudPtr cloud;
};

enum class DiagnosticLevel
{
  DEBUG,
  INFO,
  WARNING,
  ERROR,
};

struct DiagnosticEvent
{
  DiagnosticLevel level {DiagnosticLevel::INFO};
  std::string code;
  std::string message;
};

// Time is injected only where an adapter must resolve missing or invalid
// source timestamps. Mapping-engine decisions never read this port.
class ClockPort
{
public:
  virtual ~ClockPort() = default;
  virtual double nowSeconds() const = 0;
};

class SubmapStoragePort
{
public:
  virtual ~SubmapStoragePort() = default;
  virtual bool store(int index, const PointCloud & cloud) = 0;
  // A batch is one visibility transaction: load() cannot observe a prefix.
  virtual std::vector<bool> storeBatch(const std::vector<SubmapWrite> & writes) = 0;
  virtual PointCloudPtr load(int index) const = 0;
};

class DiagnosticsPort
{
public:
  virtual ~DiagnosticsPort() = default;
  virtual void emit(const DiagnosticEvent & event) = 0;
};

// Artifact identifiers are interpreted by the outer adapter. A filesystem
// adapter treats them as paths; an in-memory adapter treats them as keys.
class MapOutputPort
{
public:
  virtual ~MapOutputPort() = default;
  virtual bool prepareDirectory(const std::string & directory) = 0;
  virtual bool removeByExtension(
    const std::string & directory, const std::vector<std::string> & extensions) = 0;
  virtual bool writeBytes(const std::string & artifact, const std::string & bytes) = 0;
  virtual bool appendBytes(const std::string & artifact, const std::string & bytes) = 0;
  virtual bool writePointCloud(const std::string & artifact, const PointCloud & cloud) = 0;
  virtual bool writePointCloudXyz(
    const std::string & artifact, const PointCloudXyz & cloud) = 0;
};

struct ExternalIoPorts
{
  std::shared_ptr<ClockPort> clock;
  std::shared_ptr<SubmapStoragePort> submap_storage;
  std::shared_ptr<DiagnosticsPort> diagnostics;
  std::shared_ptr<MapOutputPort> map_output;

  void validate() const
  {
    if (!clock || !submap_storage || !diagnostics || !map_output) {
      throw std::invalid_argument("all Graph SLAM external I/O ports are required");
    }
  }
};

class ManualClock final : public ClockPort
{
public:
  explicit ManualClock(double seconds = 0.0)
  : seconds_(seconds) {}

  double nowSeconds() const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return seconds_;
  }

  void set(double seconds)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    seconds_ = seconds;
  }

private:
  mutable std::mutex mutex_;
  double seconds_;
};

class InMemorySubmapStorage final : public SubmapStoragePort
{
public:
  bool store(int index, const PointCloud & cloud) override
  {
    if (index < 0) {return false;}
    std::lock_guard<std::mutex> lock(mutex_);
    clouds_[index] = cloud;
    return true;
  }

  std::vector<bool> storeBatch(const std::vector<SubmapWrite> & writes) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<bool> results;
    results.reserve(writes.size());
    for (const auto & write : writes) {
      const bool valid = write.index >= 0 && write.cloud;
      results.push_back(valid);
      if (valid) {
        clouds_[write.index] = *write.cloud;
      }
    }
    return results;
  }

  PointCloudPtr load(int index) const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = clouds_.find(index);
    if (found == clouds_.end()) {
      return PointCloudPtr(new PointCloud);
    }
    return PointCloudPtr(new PointCloud(found->second));
  }

private:
  mutable std::mutex mutex_;
  std::map<int, PointCloud> clouds_;
};

class InMemoryDiagnostics final : public DiagnosticsPort
{
public:
  void emit(const DiagnosticEvent & event) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    events_.push_back(event);
  }

  std::vector<DiagnosticEvent> events() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return events_;
  }

private:
  mutable std::mutex mutex_;
  std::vector<DiagnosticEvent> events_;
};

class InMemoryMapOutput final : public MapOutputPort
{
public:
  bool prepareDirectory(const std::string &) override {return true;}
  bool removeByExtension(
    const std::string & directory, const std::vector<std::string> & extensions) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    eraseMatching(bytes_, directory, extensions);
    eraseMatching(clouds_, directory, extensions);
    eraseMatching(xyz_clouds_, directory, extensions);
    return true;
  }

  bool writeBytes(const std::string & artifact, const std::string & bytes) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    bytes_[artifact] = bytes;
    return true;
  }

  bool appendBytes(const std::string & artifact, const std::string & bytes) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    bytes_[artifact] += bytes;
    return true;
  }

  bool writePointCloud(const std::string & artifact, const PointCloud & cloud) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    clouds_[artifact] = cloud;
    return true;
  }

  bool writePointCloudXyz(
    const std::string & artifact, const PointCloudXyz & cloud) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    xyz_clouds_[artifact] = cloud;
    return true;
  }

  std::string bytes(const std::string & artifact) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = bytes_.find(artifact);
    return found == bytes_.end() ? std::string() : found->second;
  }

  PointCloudPtr pointCloud(const std::string & artifact) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = clouds_.find(artifact);
    if (found == clouds_.end()) {
      return PointCloudPtr(new PointCloud);
    }
    return PointCloudPtr(new PointCloud(found->second));
  }

private:
  template<typename ValueT>
  static void eraseMatching(
    std::map<std::string, ValueT> & artifacts, const std::string & directory,
    const std::vector<std::string> & extensions)
  {
    const std::string prefix = directory.empty() ? std::string() : directory + "/";
    for (auto current = artifacts.begin(); current != artifacts.end(); ) {
      const std::string & artifact = current->first;
      bool matches = prefix.empty() || artifact.compare(0, prefix.size(), prefix) == 0;
      matches = matches && std::find_if(
        extensions.begin(), extensions.end(), [&artifact](const std::string & extension) {
          return artifact.size() >= extension.size() &&
                 artifact.compare(artifact.size() - extension.size(), extension.size(),
            extension) == 0;
        }) != extensions.end();
      if (matches) {
        current = artifacts.erase(current);
      } else {
        ++current;
      }
    }
  }

  mutable std::mutex mutex_;
  std::map<std::string, std::string> bytes_;
  std::map<std::string, PointCloud> clouds_;
  std::map<std::string, PointCloudXyz> xyz_clouds_;
};

}  // namespace ports
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__EXTERNAL_IO_PORTS_HPP_
