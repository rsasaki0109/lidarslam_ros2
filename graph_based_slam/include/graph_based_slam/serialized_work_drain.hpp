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

#ifndef GRAPH_BASED_SLAM__SERIALIZED_WORK_DRAIN_HPP_
#define GRAPH_BASED_SLAM__SERIALIZED_WORK_DRAIN_HPP_

#include <mutex>

namespace graphslam
{
namespace scheduling
{

// Coalesces concurrent notifications into a single serialized drain. The
// elected caller runs another cycle after one or more notifications arrive
// while it owns the drain; other callers only mark work pending and return.
//
// Work must represent the same drain operation for every caller. It may call
// request() recursively. Exceptions release ownership and preserve any
// notification that arrived concurrently for a later request to retry.
class SerializedWorkDrain
{
public:
  SerializedWorkDrain() = default;
  SerializedWorkDrain(const SerializedWorkDrain &) = delete;
  SerializedWorkDrain & operator=(const SerializedWorkDrain &) = delete;

  template<typename WorkT>
  void request(WorkT && work)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      pending_ = true;
      if (draining_) {
        return;
      }
      draining_ = true;
    }

    try {
      while (beginCycle()) {
        work();
      }
    } catch (...) {
      std::lock_guard<std::mutex> lock(mutex_);
      draining_ = false;
      throw;
    }
  }

private:
  bool beginCycle()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!pending_) {
      draining_ = false;
      return false;
    }
    pending_ = false;
    return true;
  }

  std::mutex mutex_;
  bool draining_ {false};
  bool pending_ {false};
};

}  // namespace scheduling
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__SERIALIZED_WORK_DRAIN_HPP_
