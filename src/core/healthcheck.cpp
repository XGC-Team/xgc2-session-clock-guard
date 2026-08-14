#include "xgc_session_clock_guard/healthcheck.hpp"

#include <cmath>
#include <limits>
#include <utility>

namespace xgc_session_clock_guard {
namespace {

std::uint64_t absoluteUnsigned(std::int64_t value) {
  if (value >= 0) {
    return static_cast<std::uint64_t>(value);
  }
  if (value == std::numeric_limits<std::int64_t>::min()) {
    return static_cast<std::uint64_t>(
               std::numeric_limits<std::int64_t>::max()) +
           1U;
  }
  return static_cast<std::uint64_t>(-value);
}

} // namespace

std::string validateLockedAdmission(const AggregateAdmissionEvidence &evidence,
                                    const FrozenConfig &config,
                                    std::uint64_t expected_epoch) {
  if (evidence.session_id != config.session_id) {
    return "aggregate Session identity does not match frozen policy";
  }
  if (evidence.session_contract_sha256 != config.session_contract_sha256) {
    return "aggregate Session contract digest does not match frozen policy";
  }
  if (evidence.policy_sha256 != config.policy_sha256) {
    return "aggregate policy digest does not match verified policy bytes";
  }
  if (evidence.run_mode != config.run_mode ||
      evidence.session_time_authority !=
          toString(config.session_time_authority) ||
      evidence.clock_mapping != toString(config.clock_mapping)) {
    return "aggregate mode, authority, or mapping does not match frozen policy";
  }
  if (evidence.epoch != expected_epoch) {
    return "aggregate epoch does not match the trusted Session epoch";
  }
  if (evidence.status_sequence == 0U) {
    return "aggregate status sequence is zero";
  }
  if (evidence.state != GuardState::Locked ||
      evidence.authority_state != GuardState::Locked ||
      !evidence.authority_healthy || !evidence.sensitive_output_allowed) {
    return "aggregate Session clock admission is not locked";
  }
  if (evidence.required_routes != config.routes.size() ||
      evidence.locked_routes != config.routes.size()) {
    return "aggregate route counts do not match the frozen route set";
  }
  if (evidence.authority_age_ns > config.thresholds.max_authority_age_ns) {
    return "aggregate authority age exceeds the frozen policy";
  }
  if (config.run_mode == "simulation" || config.run_mode == "hybrid") {
    const std::string expected_topic = gazeboClockTopic(config);
    if (!evidence.gazebo_clock_required || !evidence.gazebo_clock_ready ||
        evidence.gazebo_clock_topic != expected_topic ||
        evidence.last_gazebo_clock_stamp_ns == 0U ||
        absoluteUnsigned(evidence.last_gazebo_clock_skew_ns) >
            *config.thresholds.max_gazebo_clock_skew_ns) {
      return "aggregate Gazebo authority diagnostics exceed the frozen policy";
    }
  } else if (evidence.gazebo_clock_required || evidence.gazebo_clock_ready ||
             !evidence.gazebo_clock_topic.empty() ||
             evidence.last_gazebo_clock_stamp_ns != 0U ||
             evidence.last_gazebo_clock_skew_ns != 0) {
    return "physical aggregate contains forbidden Gazebo authority diagnostics";
  }
  if (config.run_mode == "physical" || config.run_mode == "hybrid") {
    if (absoluteUnsigned(evidence.station_wall_error_ns) >
            *config.thresholds.max_station_wall_error_ns ||
        absoluteUnsigned(evidence.station_wall_step_ns) >
            *config.thresholds.max_station_wall_step_ns) {
      return "aggregate station wall diagnostics exceed the frozen policy";
    }
  } else if (evidence.station_wall_error_ns != 0 ||
             evidence.station_wall_step_ns != 0) {
    return "simulation aggregate contains forbidden station wall diagnostics";
  }
  if (config.run_mode == "hybrid") {
    if (!std::isfinite(evidence.gazebo_real_time_factor) ||
        evidence.gazebo_real_time_factor <
            *config.thresholds.min_gazebo_real_time_factor ||
        evidence.gazebo_real_time_factor >
            *config.thresholds.max_gazebo_real_time_factor) {
      return "aggregate Gazebo real-time factor exceeds the frozen policy";
    }
  } else if (evidence.gazebo_real_time_factor != 0.0) {
    return "non-Hybrid aggregate contains a Gazebo real-time factor";
  }
  if (evidence.vrpn_wire_resolution_ns != config.vrpn_wire_resolution_ns ||
      evidence.measurement_delay_enabled != config.measurement_delay_enabled ||
      evidence.delay_timestamp_policy != config.delay_timestamp_policy) {
    return "aggregate v24 timing contract does not match frozen policy";
  }
  return {};
}

std::string
validateAggregatePublisherSet(const std::vector<std::string> &publishers,
                              const std::string &expected_publisher) {
  if (publishers.size() != 1U || publishers.front() != expected_publisher) {
    return "aggregate status must have exactly one ROS master publisher "
           "named " +
           expected_publisher;
  }
  return {};
}

LiveAdmissionTracker::LiveAdmissionTracker(const FrozenConfig &config,
                                           std::uint64_t expected_epoch,
                                           std::string expected_publisher)
    : config_(&config), expected_epoch_(expected_epoch),
      expected_publisher_(std::move(expected_publisher)) {}

void LiveAdmissionTracker::observe(const std::string &publisher,
                                   const AggregateAdmissionEvidence &evidence) {
  if (!rejection_.empty() || ready()) {
    return;
  }
  if (publisher != expected_publisher_) {
    rejection_ = "aggregate status arrived from an untrusted ROS publisher";
    return;
  }
  if (accepted_samples_ != 0U && publisher != observed_publisher_) {
    rejection_ = "aggregate status samples changed publisher identity";
    return;
  }
  const std::string semantic_rejection =
      validateLockedAdmission(evidence, *config_, expected_epoch_);
  if (!semantic_rejection.empty()) {
    rejection_ = semantic_rejection;
    return;
  }
  if (accepted_samples_ != 0U && evidence.status_sequence < last_sequence_) {
    rejection_ = "aggregate status sequence regressed";
    return;
  }
  if (accepted_samples_ != 0U && evidence.status_sequence == last_sequence_) {
    return;
  }
  observed_publisher_ = publisher;
  last_sequence_ = evidence.status_sequence;
  ++accepted_samples_;
}

} // namespace xgc_session_clock_guard
