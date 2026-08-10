#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "xgc_session_clock_guard/types.hpp"

namespace xgc_session_clock_guard {

struct AggregateAdmissionEvidence {
  std::string session_id;
  std::string session_contract_sha256;
  std::string policy_sha256;
  std::string run_mode;
  std::string session_time_authority;
  std::string clock_mapping;
  std::uint64_t status_sequence{0};
  std::uint64_t epoch{0};
  GuardState state{GuardState::Initializing};
  GuardState authority_state{GuardState::Initializing};
  bool authority_healthy{false};
  bool sensitive_output_allowed{false};
  bool gazebo_clock_required{false};
  bool gazebo_clock_ready{false};
  std::string gazebo_clock_topic;
  std::uint64_t authority_age_ns{0};
  std::uint64_t last_gazebo_clock_stamp_ns{0};
  std::int64_t last_gazebo_clock_skew_ns{0};
  std::int64_t station_wall_error_ns{0};
  std::int64_t station_wall_step_ns{0};
  double gazebo_real_time_factor{0.0};
  std::uint32_t required_routes{0};
  std::uint32_t locked_routes{0};
  std::uint64_t vrpn_wire_resolution_ns{0};
  bool measurement_delay_enabled{false};
  std::string delay_timestamp_policy;
};

std::string validateLockedAdmission(const AggregateAdmissionEvidence& evidence,
                                    const FrozenConfig& config,
                                    std::uint64_t expected_epoch);

std::string validateAggregatePublisherSet(
    const std::vector<std::string>& publishers,
    const std::string& expected_publisher);

class LiveAdmissionTracker {
 public:
  LiveAdmissionTracker(const FrozenConfig& config,
                       std::uint64_t expected_epoch,
                       std::string expected_publisher);

  void observe(const std::string& publisher,
               const AggregateAdmissionEvidence& evidence);
  bool ready() const { return accepted_samples_ >= 2U && rejection_.empty(); }
  std::size_t acceptedSamples() const { return accepted_samples_; }
  const std::string& rejection() const { return rejection_; }

 private:
  const FrozenConfig* config_{nullptr};
  std::uint64_t expected_epoch_{0U};
  std::string expected_publisher_;
  std::string observed_publisher_;
  std::uint64_t last_sequence_{0U};
  std::size_t accepted_samples_{0U};
  std::string rejection_;
};

}  // namespace xgc_session_clock_guard
