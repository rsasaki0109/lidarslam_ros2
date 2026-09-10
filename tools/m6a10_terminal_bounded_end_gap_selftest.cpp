/*
 * Host-only synthetic gate for the v11 bounded end-gap contract.
 *
 * The production v10 helper has eight historical cases.  They are retained
 * here as named regressions, with three v11 cases covering the only changed
 * boundary rule.  This executable has no ROS, bag, feeder, estimator, map,
 * ground-truth, or scorer dependency.
 */
#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace {

constexpr const char *kContract =
    "m6a10-online-compute-v4-terminal-bounded-end-gap";
constexpr double kRequiredEnd = 10.0;
constexpr double kMaximumGap = 0.25;

struct Observation {
  double boundary_timestamp = 10.0;
  double trajectory_last_timestamp = 10.0;
  double trajectory_end_gap = 0.0;
  long long received_lidar = 1;
  long long received_imu = 0;
  long long received_image = 0;
  long long completed_lidar = 1;
  long long completed_imu = 0;
  long long completed_image = 0;
  long long residual_lidar = 0;
  bool buffers_observed = true;
  bool eof_observed = true;
  bool predicate_evaluated = true;
  bool quiescent = true;
  bool quiescence_observed = true;
  bool in_flight = false;
  bool completion_disposition_valid = true;
  bool support_context_proven = true;
  bool stable_polls = true;
  bool source_present = true;
};

bool finite(double value) { return std::isfinite(value); }

bool valid_locked(const Observation &o) {
  if (!finite(kRequiredEnd) || !finite(kMaximumGap) ||
      !o.buffers_observed || !o.eof_observed || !o.predicate_evaluated ||
      !o.stable_polls || !o.completion_disposition_valid ||
      !o.quiescent || !o.quiescence_observed || o.in_flight ||
      !finite(o.boundary_timestamp) || !o.source_present ||
      !o.support_context_proven)
    return false;
  if (o.completed_lidar != o.received_lidar ||
      o.completed_imu != o.received_imu ||
      o.completed_image != o.received_image ||
      o.residual_lidar != 0)
    return false;
  if (!finite(o.trajectory_last_timestamp) ||
      !finite(o.trajectory_end_gap) ||
      o.trajectory_end_gap < 0.0 ||
      o.trajectory_end_gap > kMaximumGap)
    return false;
  const double exact_gap = kRequiredEnd - o.trajectory_last_timestamp;
  return std::fabs(exact_gap - o.trajectory_end_gap) <= 1e-9;
  // Deliberately no boundary_timestamp >= kRequiredEnd check: v11 permits
  // a completed estimator boundary before required end when the exact,
  // finite, nonnegative, bounded trajectory gap proves coverage.
}

Observation base() { return Observation(); }

bool run_case(const std::string &name, bool expected, Observation o) {
  const bool actual = valid_locked(o);
  const bool pass = actual == expected;
  std::cout << "M6A10_SYNTHETIC_CASE " << name
            << " contract=" << kContract
            << " actual=" << (actual ? "pass" : "invalid")
            << " expected=" << (expected ? "pass" : "invalid")
            << " gate=" << (pass ? "PASS" : "FAIL") << '\n';
  return pass;
}

}  // namespace

int main() {
  bool all_pass = true;

  // v10 historical cases, retained verbatim by name and outcome.
  {
    Observation o = base();
    all_pass = run_case("zero_backlog", true, o) && all_pass;
  }
  {
    Observation o = base();
    o.received_imu = 1;
    o.completed_imu = 1;
    o.received_image = 1;
    o.completed_image = 1;
    all_pass = run_case("post_boundary_imu_image_tail", true, o) && all_pass;
  }
  {
    Observation o = base();
    o.received_lidar = 2;
    o.completed_lidar = 1;
    o.residual_lidar = 1;
    all_pass = run_case("residual_lidar", false, o) && all_pass;
  }
  {
    Observation o = base();
    o.support_context_proven = false;
    all_pass = run_case("pre_boundary_tail", false, o) && all_pass;
  }
  {
    Observation o = base();
    o.buffers_observed = false;
    all_pass = run_case("discard_clear", false, o) && all_pass;
  }
  {
    Observation o = base();
    o.in_flight = true;
    all_pass = run_case("active_inflight", false, o) && all_pass;
  }
  {
    Observation o = base();
    o.stable_polls = false;
    all_pass = run_case("same_rpc_stability", false, o) && all_pass;
  }
  {
    Observation o = base();
    o.eof_observed = false;
    all_pass = run_case("missing_eof", false, o) && all_pass;
  }

  // v11 bounded-end-gap cases.
  {
    Observation o = base();
    o.boundary_timestamp = 9.90;
    o.trajectory_last_timestamp = 9.90;
    o.trajectory_end_gap = 0.10;
    all_pass = run_case("bounded_end_gap", true, o) && all_pass;
  }
  {
    Observation o = base();
    o.boundary_timestamp = 9.70;
    o.trajectory_last_timestamp = 9.70;
    o.trajectory_end_gap = 0.30;
    all_pass = run_case("excessive_end_gap", false, o) && all_pass;
  }
  {
    Observation o = base();
    o.boundary_timestamp = 10.10;
    o.trajectory_last_timestamp = 10.10;
    o.trajectory_end_gap = -0.10;
    all_pass = run_case("future_boundary", false, o) && all_pass;
  }

  std::cout << "M6A10_SYNTHETIC_ALL "
            << (all_pass ? "PASS" : "FAIL") << '\n';
  return all_pass ? 0 : 1;
}
