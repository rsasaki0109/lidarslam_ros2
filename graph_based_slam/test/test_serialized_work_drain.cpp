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

#include <gtest/gtest.h>

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <stdexcept>
#include <thread>

#include "graph_based_slam/serialized_work_drain.hpp"

using graphslam::scheduling::SerializedWorkDrain;

TEST(SerializedWorkDrain, RunsOneCycleForARequest)
{
  SerializedWorkDrain drain;
  int cycles = 0;

  drain.request([&cycles]() {++cycles;});

  EXPECT_EQ(cycles, 1);
}

TEST(SerializedWorkDrain, RecursiveRequestIsCoalescedAfterTheCurrentCycle)
{
  SerializedWorkDrain drain;
  int cycles = 0;

  drain.request(
    [&]() {
      ++cycles;
      if (cycles == 1) {
        drain.request([&cycles]() {++cycles;});
      }
    });

  EXPECT_EQ(cycles, 2);
}

TEST(SerializedWorkDrain, ConcurrentRequestNeverRunsWorkConcurrentlyOrGetsLost)
{
  SerializedWorkDrain drain;
  std::mutex gate_mutex;
  std::condition_variable gate_cv;
  bool first_cycle_started = false;
  bool release_first_cycle = false;
  std::atomic<int> cycles {0};
  std::atomic<int> active_workers {0};
  std::atomic<int> maximum_active_workers {0};

  const auto work = [&]() {
      const int active = ++active_workers;
      int previous_maximum = maximum_active_workers.load();
      while (
        active > previous_maximum &&
        !maximum_active_workers.compare_exchange_weak(previous_maximum, active)) {}
      const int cycle = ++cycles;
      if (cycle == 1) {
        std::unique_lock<std::mutex> lock(gate_mutex);
        first_cycle_started = true;
        gate_cv.notify_all();
        gate_cv.wait(lock, [&release_first_cycle]() {return release_first_cycle;});
      }
      --active_workers;
    };

  std::thread owner([&]() {drain.request(work);});
  {
    std::unique_lock<std::mutex> lock(gate_mutex);
    gate_cv.wait(lock, [&first_cycle_started]() {return first_cycle_started;});
  }

  std::thread notifier([&]() {drain.request(work);});
  notifier.join();
  {
    std::lock_guard<std::mutex> lock(gate_mutex);
    release_first_cycle = true;
  }
  gate_cv.notify_all();
  owner.join();

  EXPECT_EQ(cycles.load(), 2);
  EXPECT_EQ(maximum_active_workers.load(), 1);
}

TEST(SerializedWorkDrain, ExceptionReleasesOwnershipForTheNextRequest)
{
  SerializedWorkDrain drain;
  EXPECT_THROW(
    drain.request([]() {throw std::runtime_error("drain failed");}),
    std::runtime_error);

  int cycles = 0;
  drain.request([&cycles]() {++cycles;});

  EXPECT_EQ(cycles, 1);
}
