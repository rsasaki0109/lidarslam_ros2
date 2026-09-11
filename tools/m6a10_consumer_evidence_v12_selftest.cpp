/*
 * Production-header-only v12 callback/ACK transport selftest.
 *
 * The class under test is the actual M6A10ConsumerEvidence header emitted by
 * the v12 patch chain.  No mapper, bag, ROS master, Docker, GT, or scorer is
 * started.  Host tests provide only minimal ros/std_srvs headers.
 */
#include "m6a10_consumer_evidence.h"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace {

using Evidence = M6A10ConsumerEvidence;
using Topic = Evidence::Topic;
constexpr const char *kV5 =
  "m6a10-online-compute-v5-terminal-nonlidar-at-or-after-boundary";
constexpr const char *kV4 = "m6a10-online-compute-v4-terminal-bounded-end-gap";
constexpr const char *kTransportContract =
  "m6a10-v12-callback-ack-transport-outstanding-v1";

const char *output_dir()
{
  const char *value = std::getenv("M6A10_SELFTEST_OUTPUT_DIR");
  return value == nullptr || *value == '\0' ? "/tmp" : value;
}

void set_common(const std::string &contract, const std::string &path,
                int total, int lidar, int imu, int image,
                const char *latency = "0.25")
{
  setenv("M6A10_PHASE_CONTRACT_VERSION", contract.c_str(), 1);
  setenv("M6A10_PHASE_MODE", "unpaced_ack", 1);
  setenv("M6A10_CONSUMER_EVIDENCE", path.c_str(), 1);
  setenv("M6A10_FAST_EXPECTED_MESSAGES", std::to_string(total).c_str(), 1);
  setenv("M6A10_FAST_EXPECTED_LIDAR_MESSAGES", std::to_string(lidar).c_str(), 1);
  setenv("M6A10_FAST_EXPECTED_IMU_MESSAGES", std::to_string(imu).c_str(), 1);
  setenv("M6A10_FAST_EXPECTED_IMAGE_MESSAGES", std::to_string(image).c_str(), 1);
  setenv("M6A10_FAST_MAX_CALLBACK_LATENCY_SECONDS", latency, 1);
  setenv("M6A10_FAST_MAX_BACKLOG_MESSAGES", "1", 1);
  setenv("M6A10_FAST_PACED_RATE_VERIFIED", "0", 1);
}

void callback(Evidence &evidence, Topic topic, double timestamp,
              bool accepted = true, unsigned sleep_milliseconds = 0)
{
  Evidence::CallbackScope scope = evidence.begin(topic);
  if (sleep_milliseconds != 0)
    std::this_thread::sleep_for(std::chrono::milliseconds(sleep_milliseconds));
  if (accepted)
    scope.accept(timestamp);
}

bool finish(Evidence &evidence, std::size_t lidar, std::size_t imu,
            std::size_t image, std::string *document = nullptr)
{
  std_srvs::Trigger::Request request;
  std_srvs::Trigger::Response response;
  evidence.eof_service(request, response, lidar, imu, image);
  evidence.finalize_service(request, response, lidar, imu, image);
  if (document != nullptr)
  {
    const char *path = std::getenv("M6A10_CONSUMER_EVIDENCE");
    std::ifstream stream(path == nullptr ? "" : path);
    document->assign(std::istreambuf_iterator<char>(stream),
                     std::istreambuf_iterator<char>());
  }
  return response.message == "consumer evidence PASS";
}

bool transport_contract_valid(const std::string &document)
{
  const std::string field = "\"transport_contract_version\": \"" +
    std::string(kTransportContract) + "\"";
  return document.find(field) != std::string::npos;
}

bool duplicate_ack_case()
{
  const std::string path = std::string(output_dir()) + "/m6a10_consumer_v12_duplicate.json";
  std::remove(path.c_str());
  set_common(kV5, path, 1, 0, 1, 0);
  Evidence evidence;
  callback(evidence, Topic::Imu, 1.0);
  std_srvs::Trigger::Request request;
  std_srvs::Trigger::Response response;
  evidence.ack_service(request, response);
  const bool first = response.success;
  evidence.ack_service(request, response);
  return first && !response.success;
}

bool run_case(const std::string &name)
{
  const std::string path = std::string(output_dir()) + "/m6a10_consumer_v12_" + name + ".json";
  std::remove(path.c_str());
  std::remove((path + ".eof.json").c_str());
  std::string document;
  bool expected = false;
  bool actual = false;

  if (name == "one_callback_tail81")
  {
    set_common(kV5, path, 1, 0, 1, 0);
    Evidence evidence;
    callback(evidence, Topic::Imu, 1.0);
    std_srvs::Trigger::Request request;
    std_srvs::Trigger::Response response;
    evidence.ack_service(request, response);
    expected = true;
    actual = finish(evidence, 0, 81, 0, &document) &&
      document.find("\"schema_version\": 3") != std::string::npos &&
      transport_contract_valid(document) &&
      document.find("\"transport_outstanding_at_drain\": 0") != std::string::npos &&
      document.find("\"maximum_transport_outstanding_messages\": 1") != std::string::npos &&
      document.find("\"mapper_internal_deque_current_messages\": 81") != std::string::npos &&
      document.find("\"mapper_internal_deque_peak_messages\": 81") != std::string::npos &&
      document.find("\"ack_exact\": true") != std::string::npos;
  }
  else if (name == "transport_contract_missing_or_wrong")
  {
    set_common(kV5, path, 1, 0, 1, 0);
    expected = true;
    Evidence evidence;
    callback(evidence, Topic::Imu, 1.0);
    std_srvs::Trigger::Request request;
    std_srvs::Trigger::Response response;
    evidence.ack_service(request, response);
    actual = finish(evidence, 0, 0, 0, &document) &&
      transport_contract_valid(document);
    const std::string wrong = "\"transport_contract_version\": \"wrong-contract\"";
    const std::size_t marker = document.find("transport_contract_version");
    const std::string missing = marker == std::string::npos ? document :
      document.substr(0, marker);
    actual = actual && document.find(wrong) == std::string::npos &&
      !transport_contract_valid(missing);
  }
  else if (name == "two_before_ack_peak2")
  {
    set_common(kV5, path, 2, 0, 2, 0);
    expected = true;
    Evidence evidence;
    callback(evidence, Topic::Imu, 1.0);
    callback(evidence, Topic::Imu, 1.1);
    std_srvs::Trigger::Request request;
    std_srvs::Trigger::Response response;
    evidence.ack_service(request, response);
    evidence.ack_service(request, response);
    actual = !finish(evidence, 0, 0, 0, &document) &&
      document.find("\"maximum_transport_outstanding_messages\": 2") != std::string::npos;
  }
  else if (name == "duplicate_ack_rejected")
  {
    expected = true;
    actual = duplicate_ack_case();
  }
  else if (name == "outstanding_nonzero")
  {
    set_common(kV5, path, 1, 0, 1, 0);
    expected = true;
    Evidence evidence;
    callback(evidence, Topic::Imu, 1.0);
    actual = !finish(evidence, 0, 0, 0, &document) &&
      document.find("\"transport_outstanding_at_drain\": 1") != std::string::npos;
  }
  else if (name == "count_mismatch")
  {
    set_common(kV5, path, 2, 0, 2, 0);
    expected = true;
    Evidence evidence;
    callback(evidence, Topic::Imu, 1.0);
    std_srvs::Trigger::Request request;
    std_srvs::Trigger::Response response;
    evidence.ack_service(request, response);
    actual = !finish(evidence, 0, 0, 0);
  }
  else if (name == "drop_rejected")
  {
    set_common(kV5, path, 0, 0, 0, 0);
    expected = true;
    Evidence evidence;
    callback(evidence, Topic::Imu, 1.0, false);
    actual = !finish(evidence, 0, 0, 0);
  }
  else if (name == "overflow_rejected")
  {
    set_common(kV5, path, 0, 0, 0, 0);
    expected = true;
    Evidence evidence;
    evidence.observe_queue_overflow();
    actual = !finish(evidence, 0, 0, 0);
  }
  else if (name == "callback_latency_rejected")
  {
    set_common(kV5, path, 1, 0, 1, 0, "0.0");
    expected = true;
    Evidence evidence;
    callback(evidence, Topic::Imu, 1.0, true, 2);
    std_srvs::Trigger::Request request;
    std_srvs::Trigger::Response response;
    evidence.ack_service(request, response);
    actual = !finish(evidence, 0, 0, 0);
  }
  else if (name == "v4_legacy_tail81")
  {
    set_common(kV4, path, 1, 0, 1, 0);
    expected = true;
    Evidence evidence;
    callback(evidence, Topic::Imu, 1.0);
    std_srvs::Trigger::Request request;
    std_srvs::Trigger::Response response;
    evidence.ack_service(request, response);
    actual = !finish(evidence, 0, 81, 0, &document) &&
      document.find("\"schema_version\": 2") != std::string::npos &&
      document.find("m6a10-online-compute-v2") != std::string::npos;
  }
  else if (name == "unknown_contract_disabled")
  {
    set_common("m6a10-online-compute-unknown", path, 0, 0, 0, 0);
    Evidence evidence;
    expected = true;
    actual = !evidence.enabled();
  }
  else
  {
    std::cerr << "unknown case: " << name << '\n';
    return false;
  }

  const bool gate = actual == expected;
  std::cout << "M6A10_CONSUMER_CASE " << name
            << " actual=" << (actual ? "pass" : "invalid")
            << " expected=" << (expected ? "pass" : "invalid")
            << " gate=" << (gate ? "PASS" : "FAIL") << '\n';
  return gate;
}

}  // namespace

int main()
{
  const std::vector<std::string> cases = {
    "one_callback_tail81", "two_before_ack_peak2", "duplicate_ack_rejected",
    "transport_contract_missing_or_wrong",
    "outstanding_nonzero", "count_mismatch", "drop_rejected",
    "overflow_rejected", "callback_latency_rejected", "v4_legacy_tail81",
    "unknown_contract_disabled"};
  bool all_pass = true;
  for (const std::string &name : cases)
    all_pass = run_case(name) && all_pass;
  std::cout << "M6A10_CONSUMER_ALL " << (all_pass ? "PASS" : "FAIL") << '\n';
  return all_pass ? 0 : 1;
}
