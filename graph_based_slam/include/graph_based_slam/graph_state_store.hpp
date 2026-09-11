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

#ifndef GRAPH_BASED_SLAM__GRAPH_STATE_STORE_HPP_
#define GRAPH_BASED_SLAM__GRAPH_STATE_STORE_HPP_

#include <cstddef>
#include <mutex>
#include <utility>

#include <lidarslam_msgs/msg/map_array.hpp>
#include <lidarslam_msgs/msg/sub_map.hpp>
#include <std_msgs/msg/header.hpp>

namespace graphslam
{

// The ROS shell's authoritative ROS-message input snapshot. Expensive cloud
// conversion and cache I/O must finish before replace()/append() so readers
// observe either the previous complete state or the next complete state.
// Mapping workflow state and accepted loop edges belong to GraphSlamApplication.
class GraphStateStore
{
public:
  void replace(lidarslam_msgs::msg::MapArray map_array)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    map_array_ = std::move(map_array);
    initialized_ = true;
  }

  std::size_t append(
    lidarslam_msgs::msg::SubMap submap,
    const std_msgs::msg::Header & map_header)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    map_array_.header = map_header;
    map_array_.submaps.push_back(std::move(submap));
    initialized_ = true;
    return map_array_.submaps.size();
  }

  std::size_t submapCount() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return map_array_.submaps.size();
  }

  bool snapshot(lidarslam_msgs::msg::MapArray & map_array) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_) {
      return false;
    }
    map_array = map_array_;
    return true;
  }

private:
  mutable std::mutex mutex_;
  bool initialized_{false};
  lidarslam_msgs::msg::MapArray map_array_;
};

}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__GRAPH_STATE_STORE_HPP_
