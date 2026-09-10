#include <memory>

#include <glim/mapping/sub_mapping.hpp>
#include <glim/odometry/odometry_estimation_cpu.hpp>
#include <glim/preprocess/cloud_preprocessor.hpp>
#include <glim/util/raw_points.hpp>

#ifndef GLIM_CLEAN_ROOM_EXPECTED_COMMIT
#error "CMake must bind an immutable GLIM core commit"
#endif

int main() {
  // These are the public core types used by the host-owned adapter.  No ROS
  // bridge header, class, plugin, or source path is involved here.
  glim::RawPoints::Ptr raw = std::make_shared<glim::RawPoints>();
  glim::CloudPreprocessor preprocessor;
  glim::OdometryEstimationCPU odometry;
  glim::SubMapping mapping;
  return (raw && &preprocessor && &odometry && &mapping) ? 0 : 1;
}
