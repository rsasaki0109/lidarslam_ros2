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

#ifndef GRAPH_BASED_SLAM__REGISTRATION_FACTORY_HPP_
#define GRAPH_BASED_SLAM__REGISTRATION_FACTORY_HPP_

// The loop-closure registration construction shared by the ROS component
// and the offline deterministic runner, so both verify candidates with
// byte-identical registration settings (docs/roadmap/v0.6.md, Phase 2).

#include <pcl/memory.h>  // NOLINT(build/include_order)
#include <pcl/registration/registration.h>  // NOLINT(build/include_order)
#include <pclomp/gicp_omp.h>  // NOLINT(build/include_order)
#include <pclomp/ndt_omp.h>  // NOLINT(build/include_order)

#include <string>

#include <pclomp/gicp_omp_impl.hpp>
#include <pclomp/ndt_omp_impl.hpp>
#include <pclomp/voxel_grid_covariance_omp_impl.hpp>

namespace graphslam
{
namespace backend_core
{

// Returns nullptr for an unknown method; the caller decides how to fail.
inline pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr
makeLoopRegistration(const std::string & method, double ndt_resolution, int ndt_num_threads)
{
  if (method == "NDT") {
    auto ndt = pcl::make_shared<
      pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>>();
    ndt->setMaximumIterations(100);
    ndt->setResolution(ndt_resolution);
    ndt->setTransformationEpsilon(0.01);
    ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    if (ndt_num_threads > 0) {ndt->setNumThreads(ndt_num_threads);}
    return ndt;
  }
  if (method == "GICP") {
    auto gicp = pcl::make_shared<
      pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>>();
    gicp->setMaxCorrespondenceDistance(30);
    gicp->setMaximumIterations(100);
    gicp->setTransformationEpsilon(1e-8);
    gicp->setEuclideanFitnessEpsilon(1e-6);
    gicp->setRANSACIterations(0);
    return gicp;
  }
  return nullptr;
}

}  // namespace backend_core
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__REGISTRATION_FACTORY_HPP_
