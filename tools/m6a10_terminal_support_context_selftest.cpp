/*
 * Benchmark-only synthetic gate for the v10 terminal support-context helper.
 *
 * This executable deliberately exercises the production header-only emitter
 * and writes the exact consumer documents that the host compositor consumes.
 * It has no bag, ROS topic, feeder, ground-truth, or scorer dependency.
 */
#include "m6a10_terminal_support_context.h"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace {

using Context = M6A10TerminalSupportContext;
using Topic = Context::Topic;

const char *output_dir()
{
  const char *value = std::getenv("M6A10_SELFTEST_OUTPUT_DIR");
  return value == nullptr || *value == '\0' ? "/dev/shm" : value;
}

void accept(Context &context, Topic topic, double timestamp)
{
  Context::CallbackScope scope = context.begin(topic);
  scope.accept(timestamp);
}

void complete(Context &context, const std::vector<Topic> &topics,
              double timestamp)
{
  context.begin_synchronization_unit();
  for (Topic topic : topics)
    context.observe_queue_pop(topic, true);
  context.begin_estimator();
  context.observe_completed_estimator(timestamp);
}

void poll_twice(Context &context, bool delayed)
{
  context.snapshot_pending_buffers();
  if (delayed)
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
  context.snapshot_pending_buffers();
}

bool finalize(Context &context, bool prove_support_context,
              bool delayed_poll)
{
  context.observe_terminal_eof();
  const bool predicate =
    context.observe_terminal_sync_predicate(false);
  if (prove_support_context && predicate)
  {
    for (const std::string &record_id : context.pending_record_ids())
      context.prove_terminal_record(record_id, true, false,
                                    "strictly_after_completed_boundary");
  }
  poll_twice(context, delayed_poll);
  std_srvs::Trigger::Request request;
  std_srvs::Trigger::Response response;
  context.finalize_service(request, response);
  return response.message == "terminal support-context evidence PASS";
}

bool run_case(const std::string &name)
{
  const std::string path = std::string(output_dir()) +
    "/m6a10_terminal_" + name + ".json";
  std::remove(path.c_str());
  setenv("M6A10_PHASE_CONTRACT_VERSION", Context::contract_version(), 1);
  setenv("M6A10_PHASE_MODE", "unpaced_ack", 1);
  setenv("M6A10_TERMINAL_SUPPORT_CONTEXT_EVIDENCE", path.c_str(), 1);
  // These are synthetic identities, independent of the preregistered bag.
  setenv("M6A10_FAST_REQUIRED_END_TIMESTAMP_SECONDS", "10.0", 1);
  setenv("M6A10_FAST_MAX_END_GAP_SECONDS", "0.25", 1);
  setenv("M6A10_FAST_MIN_TERMINAL_POLL_WALL_SECONDS", "0.05", 1);

  Context context;
  bool actual_pass = false;
  bool expected_pass = false;

  if (name == "zero_backlog")
  {
    accept(context, Topic::Lidar, 10.0);
    complete(context, {Topic::Lidar}, 10.0);
    expected_pass = true;
    actual_pass = finalize(context, false, true);
  }
  else if (name == "post_boundary_imu_image_tail")
  {
    accept(context, Topic::Lidar, 10.0);
    accept(context, Topic::Imu, 10.01);
    accept(context, Topic::Image, 10.02);
    complete(context, {Topic::Lidar}, 10.0);
    expected_pass = true;
    actual_pass = finalize(context, true, true);
  }
  else if (name == "residual_lidar")
  {
    accept(context, Topic::Lidar, 10.0);
    accept(context, Topic::Lidar, 10.01);
    complete(context, {Topic::Lidar}, 10.0);
    actual_pass = finalize(context, true, true);
  }
  else if (name == "pre_boundary_tail")
  {
    accept(context, Topic::Lidar, 10.0);
    accept(context, Topic::Imu, 9.99);
    complete(context, {Topic::Lidar}, 10.0);
    actual_pass = finalize(context, true, true);
  }
  else if (name == "discard_clear")
  {
    accept(context, Topic::Lidar, 10.0);
    context.observe_queue_clear(Topic::Lidar);
    actual_pass = finalize(context, false, true);
  }
  else if (name == "active_inflight")
  {
    accept(context, Topic::Lidar, 10.0);
    context.begin_synchronization_unit();
    context.observe_queue_pop(Topic::Lidar, true);
    context.begin_estimator();
    actual_pass = finalize(context, false, true);
  }
  else if (name == "same_rpc_stability")
  {
    accept(context, Topic::Lidar, 10.0);
    complete(context, {Topic::Lidar}, 10.0);
    context.observe_terminal_eof();
    context.observe_terminal_sync_predicate(false);
    // Two snapshots in one RPC/immediate succession must not stabilize.
    context.snapshot_pending_buffers();
    context.snapshot_pending_buffers();
    std_srvs::Trigger::Request request;
    std_srvs::Trigger::Response response;
    context.finalize_service(request, response);
    actual_pass = response.message == "terminal support-context evidence PASS";
  }
  else if (name == "missing_eof")
  {
    accept(context, Topic::Lidar, 10.0);
    complete(context, {Topic::Lidar}, 10.0);
    context.observe_terminal_sync_predicate(false);
    context.snapshot_pending_buffers();
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
    context.snapshot_pending_buffers();
    std_srvs::Trigger::Request request;
    std_srvs::Trigger::Response response;
    context.finalize_service(request, response);
    actual_pass = response.message == "terminal support-context evidence PASS";
  }
  else
  {
    std::cerr << "unknown synthetic case: " << name << '\n';
    return false;
  }

  const bool expected = actual_pass == expected_pass;
  std::cout << "M6A10_SYNTHETIC_CASE " << name
            << " actual=" << (actual_pass ? "pass" : "invalid")
            << " expected=" << (expected_pass ? "pass" : "invalid")
            << " gate=" << (expected ? "PASS" : "FAIL") << '\n';
  return expected;
}

}  // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "m6a10_terminal_support_context_selftest",
            ros::init_options::AnonymousName |
              ros::init_options::NoSigintHandler);
  const std::vector<std::string> cases = {
    "zero_backlog", "post_boundary_imu_image_tail", "residual_lidar",
    "pre_boundary_tail", "discard_clear", "active_inflight",
    "same_rpc_stability", "missing_eof"};
  bool all_pass = true;
  for (const std::string &name : cases)
    all_pass = run_case(name) && all_pass;
  std::cout << "M6A10_SYNTHETIC_ALL " << (all_pass ? "PASS" : "FAIL") << '\n';
  return all_pass ? 0 : 1;
}
