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

#ifndef FILESYSTEM_IO_PORTS_HPP_
#define FILESYSTEM_IO_PORTS_HPP_

#include <memory>
#include <string>
#include <vector>

#include "graph_based_slam/external_io_ports.hpp"

namespace graphslam
{
namespace adapters
{

// These are outer adapters. Their filesystem/PCL I/O dependencies must never
// be linked into graph_slam_application.
class PcdSubmapStorage final : public ports::SubmapStoragePort
{
public:
  explicit PcdSubmapStorage(std::string directory);
  bool store(int index, const ports::PointCloud & cloud) override;
  std::vector<bool> storeBatch(const std::vector<ports::SubmapWrite> & writes) override;
  ports::PointCloudPtr load(int index) const override;

private:
  std::string directory_;
  mutable std::mutex mutex_;
};

class FilesystemMapOutput final : public ports::MapOutputPort
{
public:
  bool prepareDirectory(const std::string & directory) override;
  bool removeByExtension(
    const std::string & directory, const std::vector<std::string> & extensions) override;
  bool writeBytes(const std::string & artifact, const std::string & bytes) override;
  bool appendBytes(const std::string & artifact, const std::string & bytes) override;
  bool writePointCloud(
    const std::string & artifact, const ports::PointCloud & cloud) override;
  bool writePointCloudXyz(
    const std::string & artifact, const ports::PointCloudXyz & cloud) override;
};

}  // namespace adapters
}  // namespace graphslam

#endif  // FILESYSTEM_IO_PORTS_HPP_
