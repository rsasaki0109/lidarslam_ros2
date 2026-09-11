/*
 * v17 production-header selftest.
 *
 * This executable includes the exact post-v17 production header.  It does
 * not reimplement the terminal predicate, start ROS, open a bag, use Docker,
 * read ground truth, or invoke a scorer.  The cases exercise the production
 * synchronization ledger directly: an empty retry is harmless, while a
 * retry after a record has been incorporated remains invalid.
 */
#include "m6a10_terminal_support_context.h"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
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

void configure(const std::string &path)
{
  setenv("M6A10_PHASE_CONTRACT_VERSION", kContract, 1);
  setenv("M6A10_PHASE_MODE", "unpaced_ack", 1);
  setenv("M6A10_TERMINAL_SUPPORT_CONTEXT_EVIDENCE", path.c_str(), 1);
  setenv("M6A10_FAST_REQUIRED_END_TIMESTAMP_SECONDS", "10.0", 1);
  setenv("M6A10_FAST_MAX_END_GAP_SECONDS", "0.25", 1);
  setenv("M6A10_FAST_MIN_TERMINAL_POLL_WALL_SECONDS", "0.05", 1);
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

bool finalize(Context &context, bool observe_eof = true,
              bool observe_predicate = true, bool can_form = false,
              bool prove_records = true, const char *reason = kStrictReason)
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
  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  context.snapshot_pending_buffers();
  std_srvs::Trigger::Request request;
  std_srvs::Trigger::Response response;
  context.finalize_service(request, response);
  return response.message == "terminal support-context evidence PASS";
}

bool evidence_is_pass(const std::string &path)
{
  std::ifstream stream(path.c_str());
  std::string contents((std::istreambuf_iterator<char>(stream)),
                       std::istreambuf_iterator<char>());
  return contents.find("\"status\": \"pass\"") != std::string::npos &&
    contents.find("ground_truth_content_opened\": false") != std::string::npos &&
    contents.find("scorer_invoked\": false") != std::string::npos;
}

bool run_case(const std::string &name)
{
  const std::string path = std::string(output_dir()) +
    "/m6a10_terminal_v17_production_" + name + ".json";
  std::remove(path.c_str());
  std::remove((path + ".part").c_str());
  configure(path);
  if (std::string(Context::contract_version()) != kContract)
    return false;

  Context context;
  bool expected_pass = false;
  bool actual_pass = false;
  if (name == "empty_retry_then_complete")
  {
    // This is the production retry boundary: no callback has been acquired.
    context.begin_synchronization_unit();
    context.abort_retryable_synchronization_unit();
    accept(context, Topic::Lidar, 10.0);
    complete(context, {Topic::Lidar}, 10.0);
    expected_pass = true;
    actual_pass = finalize(context);
  }
  else if (name == "strict_after_nonlidar")
  {
    accept(context, Topic::Lidar, 10.0);
    accept(context, Topic::Image, 10.01);
    complete(context, {Topic::Lidar}, 10.0);
    expected_pass = true;
    actual_pass = finalize(context);
  }
  else if (name == "equal_boundary_nonlidar")
  {
    accept(context, Topic::Lidar, 10.0);
    accept(context, Topic::Image, 10.0);
    complete(context, {Topic::Lidar}, 10.0);
    expected_pass = true;
    actual_pass = finalize(context);
  }
  else if (name == "no_candidate_retry")
  {
    context.begin_synchronization_unit();
    context.abort_retryable_synchronization_unit();
    expected_pass = false;
    actual_pass = finalize(context, true, false, false, false);
  }
  else if (name == "partial_retry_after_record")
  {
    accept(context, Topic::Lidar, 10.0);
    context.begin_synchronization_unit();
    context.observe_queue_pop(Topic::Lidar, true);
    context.abort_retryable_synchronization_unit();
    expected_pass = false;
    actual_pass = finalize(context, true, false, false, false);
  }
  else if (name == "explicit_discard")
  {
    accept(context, Topic::Lidar, 10.0);
    context.begin_synchronization_unit();
    context.observe_queue_pop(Topic::Lidar, false);
    context.abort_synchronization_unit();
    expected_pass = false;
    actual_pass = finalize(context, true, false, false, false);
  }
  else if (name == "processing_failure")
  {
    accept(context, Topic::Lidar, 10.0);
    context.begin_synchronization_unit();
    // Starting an estimator with no incorporated record is a production
    // processing failure and must remain sticky invalid.
    context.begin_estimator();
    expected_pass = false;
    actual_pass = finalize(context, true, false, false, false);
  }
  else
  {
    std::cerr << "unknown v17 production case: " << name << '\n';
    return false;
  }
  const bool gate = actual_pass == expected_pass;
  if (expected_pass && gate && !evidence_is_pass(path))
    return false;
  std::cout << "M6A10_V17_PRODUCTION_CASE " << name
            << " actual=" << (actual_pass ? "pass" : "invalid")
            << " expected=" << (expected_pass ? "pass" : "invalid")
            << " gate=" << (gate ? "PASS" : "FAIL") << '\n';
  return gate;
}

}  // namespace

int main()
{
  const std::vector<std::string> cases = {
    "empty_retry_then_complete", "strict_after_nonlidar",
    "equal_boundary_nonlidar", "no_candidate_retry",
    "partial_retry_after_record", "explicit_discard", "processing_failure"};
  bool all_pass = true;
  for (const std::string &name : cases)
    all_pass = run_case(name) && all_pass;
  std::cout << "M6A10_V17_PRODUCTION_ALL "
            << (all_pass ? "PASS" : "FAIL") << '\n';
  return all_pass ? 0 : 1;
}
