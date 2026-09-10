/*
 * Production-header-only v12 terminal selftest.
 *
 * This executable includes the exact patched
 * M6A10TerminalSupportContext header.  It does not reimplement the terminal
 * predicate, open a bag, start ROS, use Docker, read GT, or invoke a scorer.
 * Host tests provide only minimal ros/std_srvs headers so the production
 * header can be compiled with -Werror on a machine without ROS installed.
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
constexpr const char *kContract =
  "m6a10-online-compute-v5-terminal-nonlidar-at-or-after-boundary";
constexpr const char *kStrictReason = "strictly_after_completed_boundary";

const char *output_dir()
{
  const char *value = std::getenv("M6A10_SELFTEST_OUTPUT_DIR");
  return value == nullptr || *value == '\0' ? "/tmp" : value;
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

bool terminal_finalize(Context &context, bool observe_eof = true,
                       bool observe_predicate = true, bool can_form = false,
                       bool prove_records = true,
                       const char *reason = kStrictReason,
                       bool delayed_poll = true)
{
  if (observe_eof) context.observe_terminal_eof();
  const bool predicate = observe_predicate &&
    context.observe_terminal_sync_predicate(can_form);
  if (prove_records && predicate)
  {
    for (const std::string &record_id : context.pending_record_ids())
      context.prove_terminal_record(record_id, true, can_form, reason);
  }
  context.snapshot_pending_buffers();
  if (delayed_poll)
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
  context.snapshot_pending_buffers();
  std_srvs::Trigger::Request request;
  std_srvs::Trigger::Response response;
  context.finalize_service(request, response);
  return response.message == "terminal support-context evidence PASS";
}

bool run_case(const std::string &name)
{
  const std::string path = std::string(output_dir()) +
    "/m6a10_terminal_v12_" + name + ".json";
  std::remove(path.c_str());
  std::remove((path + ".part").c_str());
  if (std::string(Context::contract_version()) != kContract)
    return false;
  setenv("M6A10_PHASE_CONTRACT_VERSION", kContract, 1);
  setenv("M6A10_PHASE_MODE", "unpaced_ack", 1);
  setenv("M6A10_TERMINAL_SUPPORT_CONTEXT_EVIDENCE", path.c_str(), 1);
  setenv("M6A10_FAST_REQUIRED_END_TIMESTAMP_SECONDS", "10.0", 1);
  setenv("M6A10_FAST_MAX_END_GAP_SECONDS", "0.25", 1);
  setenv("M6A10_FAST_MIN_TERMINAL_POLL_WALL_SECONDS", "0.05", 1);

  Context context;
  bool expected_pass = false;
  bool actual_pass = false;
  if (name == "zero_backlog")
  {
    accept(context, Topic::Lidar, 10.0);
    complete(context, {Topic::Lidar}, 10.0);
    expected_pass = true;
    actual_pass = terminal_finalize(context);
  }
  else if (name == "post_boundary_imu_image_tail")
  {
    accept(context, Topic::Lidar, 10.0);
    accept(context, Topic::Imu, 10.01);
    accept(context, Topic::Image, 10.02);
    complete(context, {Topic::Lidar}, 10.0);
    expected_pass = true;
    actual_pass = terminal_finalize(context);
  }
  else if (name == "equal_boundary_nonlidar")
  {
    accept(context, Topic::Lidar, 10.0);
    accept(context, Topic::Image, 10.0);
    complete(context, {Topic::Lidar}, 10.0);
    expected_pass = true;
    actual_pass = terminal_finalize(context);
  }
  else if (name == "strict_after_reason")
  {
    accept(context, Topic::Lidar, 10.0);
    accept(context, Topic::Image, 10.01);
    complete(context, {Topic::Lidar}, 10.0);
    expected_pass = true;
    actual_pass = terminal_finalize(context);
  }
  else if (name == "equal_boundary_lidar")
  {
    accept(context, Topic::Lidar, 10.0);
    accept(context, Topic::Lidar, 10.0);
    complete(context, {Topic::Lidar}, 10.0);
    actual_pass = terminal_finalize(context);
  }
  else if (name == "pre_boundary_tail")
  {
    accept(context, Topic::Lidar, 10.0);
    accept(context, Topic::Imu, 9.99);
    complete(context, {Topic::Lidar}, 10.0);
    actual_pass = terminal_finalize(context);
  }
  else if (name == "forming_unit")
  {
    accept(context, Topic::Lidar, 10.0);
    accept(context, Topic::Image, 10.01);
    complete(context, {Topic::Lidar}, 10.0);
    actual_pass = terminal_finalize(context, true, true, true);
  }
  else if (name == "predicate_unproven")
  {
    accept(context, Topic::Lidar, 10.0);
    accept(context, Topic::Image, 10.01);
    complete(context, {Topic::Lidar}, 10.0);
    actual_pass = terminal_finalize(context, true, false, false, false);
  }
  else if (name == "wrong_reason")
  {
    accept(context, Topic::Lidar, 10.0);
    accept(context, Topic::Image, 10.01);
    complete(context, {Topic::Lidar}, 10.0);
    actual_pass = terminal_finalize(context, true, true, false, true,
                                    "wrong_reason");
  }
  else if (name == "discard_clear")
  {
    accept(context, Topic::Lidar, 10.0);
    accept(context, Topic::Image, 10.01);
    complete(context, {Topic::Lidar}, 10.0);
    context.observe_queue_clear(Topic::Image);
    actual_pass = terminal_finalize(context);
  }
  else if (name == "active_inflight")
  {
    accept(context, Topic::Lidar, 10.0);
    context.begin_synchronization_unit();
    context.observe_queue_pop(Topic::Lidar, true);
    context.begin_estimator();
    actual_pass = terminal_finalize(context);
  }
  else if (name == "same_rpc_stability")
  {
    accept(context, Topic::Lidar, 10.0);
    complete(context, {Topic::Lidar}, 10.0);
    context.observe_terminal_eof();
    context.observe_terminal_sync_predicate(false);
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
    actual_pass = terminal_finalize(context, false, false, false, false);
  }
  else if (name == "bounded_end_gap")
  {
    accept(context, Topic::Lidar, 9.90);
    complete(context, {Topic::Lidar}, 9.90);
    expected_pass = true;
    actual_pass = terminal_finalize(context);
  }
  else if (name == "excessive_end_gap")
  {
    accept(context, Topic::Lidar, 9.70);
    complete(context, {Topic::Lidar}, 9.70);
    actual_pass = terminal_finalize(context);
  }
  else if (name == "future_boundary")
  {
    accept(context, Topic::Lidar, 10.10);
    complete(context, {Topic::Lidar}, 10.10);
    actual_pass = terminal_finalize(context);
  }
  else
  {
    std::cerr << "unknown synthetic case: " << name << '\n';
    return false;
  }
  const bool gate = actual_pass == expected_pass;
  std::cout << "M6A10_SYNTHETIC_CASE " << name
            << " contract=" << Context::contract_version()
            << " actual=" << (actual_pass ? "pass" : "invalid")
            << " expected=" << (expected_pass ? "pass" : "invalid")
            << " gate=" << (gate ? "PASS" : "FAIL") << '\n';
  return gate;
}

}  // namespace

int main()
{
  const std::vector<std::string> cases = {
    "zero_backlog", "post_boundary_imu_image_tail",
    "equal_boundary_nonlidar", "strict_after_reason", "equal_boundary_lidar",
    "pre_boundary_tail", "forming_unit", "predicate_unproven", "wrong_reason",
    "discard_clear", "active_inflight", "same_rpc_stability", "missing_eof",
    "bounded_end_gap", "excessive_end_gap", "future_boundary"};
  bool all_pass = true;
  for (const std::string &name : cases)
    all_pass = run_case(name) && all_pass;
  std::cout << "M6A10_SYNTHETIC_ALL " << (all_pass ? "PASS" : "FAIL") << '\n';
  return all_pass ? 0 : 1;
}
