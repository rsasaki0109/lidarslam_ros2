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

#include <gtest/gtest.h>

#include "graph_based_slam/loop_search_schedule.hpp"

TEST(LoopSearchSchedule, StrideOneSearchesEveryNonNegativeQuery)
{
  EXPECT_FALSE(graphslam::loop_search_schedule::shouldSearch(-1, 1));
  EXPECT_TRUE(graphslam::loop_search_schedule::shouldSearch(0, 1));
  EXPECT_TRUE(graphslam::loop_search_schedule::shouldSearch(17, 1));
}

TEST(LoopSearchSchedule, StrideFiveUsesOneBasedSubmapMultiples)
{
  for (int query = 0; query < 15; ++query) {
    const bool expected = query == 4 || query == 9 || query == 14;
    EXPECT_EQ(graphslam::loop_search_schedule::shouldSearch(query, 5), expected)
      << "query=" << query;
  }
}

TEST(LoopSearchSchedule, InvalidStrideFallsBackToEveryQuery)
{
  EXPECT_TRUE(graphslam::loop_search_schedule::shouldSearch(0, 0));
  EXPECT_TRUE(graphslam::loop_search_schedule::shouldSearch(3, -4));
}
