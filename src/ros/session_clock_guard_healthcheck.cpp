#include <chrono>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <boost/function.hpp>
#include <ros/master.h>
#include <ros/message_event.h>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <ros/this_node.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <xgc_session_clock_guard/ClockGuardAggregateStatus.h>

#include "xgc_session_clock_guard/epoch_fence.hpp"
#include "xgc_session_clock_guard/frozen_config.hpp"
#include "xgc_session_clock_guard/healthcheck.hpp"
#include "xgc_session_clock_guard/types.hpp"

namespace xgc_session_clock_guard {
namespace {

constexpr const char *kAggregateTopic = "/xgc/session/clock/status";
constexpr const char *kExpectedPublisher = "/xgc_session_clock_guard";

AggregateAdmissionEvidence
evidenceFromStatus(const ClockGuardAggregateStatus &status) {
  AggregateAdmissionEvidence evidence;
  evidence.session_id = status.session_id;
  evidence.session_contract_sha256 = status.session_contract_sha256;
  evidence.policy_sha256 = status.policy_sha256;
  evidence.run_mode = status.run_mode;
  evidence.session_time_authority = status.session_time_authority;
  evidence.clock_mapping = status.clock_mapping;
  evidence.status_sequence = status.status_sequence;
  evidence.epoch = status.epoch;
  evidence.state = static_cast<GuardState>(status.state);
  evidence.authority_state = static_cast<GuardState>(status.authority_state);
  evidence.authority_healthy = status.authority_healthy;
  evidence.sensitive_output_allowed = status.sensitive_output_allowed;
  evidence.gazebo_clock_required = status.gazebo_clock_required;
  evidence.gazebo_clock_ready = status.gazebo_clock_ready;
  evidence.gazebo_clock_topic = status.gazebo_clock_topic;
  evidence.authority_age_ns = status.authority_age_ns;
  evidence.last_gazebo_clock_stamp_ns = status.last_gazebo_clock_stamp_ns;
  evidence.last_gazebo_clock_skew_ns = status.last_gazebo_clock_skew_ns;
  evidence.station_wall_error_ns = status.station_wall_error_ns;
  evidence.station_wall_step_ns = status.station_wall_step_ns;
  evidence.gazebo_real_time_factor = status.gazebo_real_time_factor;
  evidence.required_routes = status.required_routes;
  evidence.locked_routes = status.locked_routes;
  evidence.vrpn_wire_resolution_ns = status.vrpn_wire_resolution_ns;
  evidence.measurement_delay_enabled = status.measurement_delay_enabled;
  evidence.delay_timestamp_policy = status.delay_timestamp_policy;
  return evidence;
}

std::vector<std::string> publishersForTopic(const std::string &topic) {
  XmlRpc::XmlRpcValue request;
  XmlRpc::XmlRpcValue response;
  XmlRpc::XmlRpcValue payload;
  request.setSize(1);
  request[0] = ros::this_node::getName();
  if (!ros::master::execute("getSystemState", request, response, payload,
                            false)) {
    throw std::runtime_error("ROS master getSystemState failed");
  }
  if (payload.getType() != XmlRpc::XmlRpcValue::TypeArray ||
      payload.size() != 3 ||
      payload[0].getType() != XmlRpc::XmlRpcValue::TypeArray) {
    throw std::runtime_error("ROS master returned malformed system state");
  }

  std::vector<std::string> result;
  XmlRpc::XmlRpcValue &publications = payload[0];
  for (int index = 0; index < publications.size(); ++index) {
    XmlRpc::XmlRpcValue &entry = publications[index];
    if (entry.getType() != XmlRpc::XmlRpcValue::TypeArray ||
        entry.size() != 2 ||
        entry[0].getType() != XmlRpc::XmlRpcValue::TypeString ||
        entry[1].getType() != XmlRpc::XmlRpcValue::TypeArray) {
      throw std::runtime_error("ROS master returned malformed publisher entry");
    }
    if (static_cast<std::string>(entry[0]) != topic) {
      continue;
    }
    XmlRpc::XmlRpcValue &nodes = entry[1];
    for (int node_index = 0; node_index < nodes.size(); ++node_index) {
      if (nodes[node_index].getType() != XmlRpc::XmlRpcValue::TypeString) {
        throw std::runtime_error(
            "ROS master returned malformed publisher name");
      }
      result.push_back(static_cast<std::string>(nodes[node_index]));
    }
  }
  return result;
}

void requireUniquePublisher() {
  const std::string rejection = validateAggregatePublisherSet(
      publishersForTopic(kAggregateTopic), kExpectedPublisher);
  if (!rejection.empty()) {
    throw std::runtime_error(rejection);
  }
}

} // namespace
} // namespace xgc_session_clock_guard

int main(int argc, char **argv) {
  if (argc != 4) {
    std::cerr << "usage: session_clock_guard_healthcheck <policy_file> "
                 "<policy_sha256> <epoch_id>\n";
    return 2;
  }
  try {
    const std::string policy_file = argv[1];
    const std::string policy_sha256 = argv[2];
    const std::uint64_t expected_epoch =
        xgc_session_clock_guard::parseEpochId(argv[3]);
    const auto config = xgc_session_clock_guard::FrozenConfigLoader::loadFile(
        policy_file, policy_sha256);
    xgc_session_clock_guard::EpochFenceLease epoch_fence_lease(policy_file);
    xgc_session_clock_guard::loadAndValidateEpochFence(policy_file, config,
                                                       expected_epoch);

    ros::init(argc, argv, "xgc_session_clock_guard_healthcheck",
              ros::init_options::AnonymousName |
                  ros::init_options::NoSigintHandler);
    ros::master::setRetryTimeout(ros::WallDuration(0.5));
    xgc_session_clock_guard::requireUniquePublisher();

    ros::NodeHandle node;
    xgc_session_clock_guard::LiveAdmissionTracker tracker(
        config, expected_epoch, "/xgc_session_clock_guard");
    using StatusEvent = ros::MessageEvent<
        xgc_session_clock_guard::ClockGuardAggregateStatus const>;
    const boost::function<void(const StatusEvent &)> status_callback =
        [&tracker](const StatusEvent &event) {
          tracker.observe(
              event.getPublisherName(),
              xgc_session_clock_guard::evidenceFromStatus(*event.getMessage()));
        };
    ros::SubscribeOptions status_options;
    status_options.initByFullCallbackType<const StatusEvent &>(
        "/xgc/session/clock/status", 4U, status_callback);
    const auto subscriber = node.subscribe(status_options);
    (void)subscriber;

    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (ros::ok() && !tracker.ready() && tracker.rejection().empty() &&
           std::chrono::steady_clock::now() < deadline) {
      ros::spinOnce();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (!tracker.rejection().empty()) {
      std::cerr << tracker.rejection() << '\n';
      return 1;
    }
    if (!tracker.ready()) {
      std::cerr << "Session Clock Guard readiness requires two strictly "
                   "advancing live aggregate samples from its sole publisher\n";
      return 1;
    }

    xgc_session_clock_guard::requireUniquePublisher();
    epoch_fence_lease.validateCurrent();
    xgc_session_clock_guard::loadAndValidateEpochFence(policy_file, config,
                                                       expected_epoch);
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "Session Clock Guard health check failed closed: "
              << error.what() << '\n';
    return 2;
  }
}
