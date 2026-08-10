#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace xgc_session_clock_guard {

enum class GuardState : std::uint8_t {
  Initializing = 0,
  Locked = 1,
  Degraded = 2,
  Lost = 3,
};

enum class SourceDomain : std::uint8_t {
  Simulation = 0,
  Physical = 1,
};

enum class StreamKind : std::uint8_t {
  Pose = 0,
  Twist = 1,
};

enum class SessionTimeAuthority : std::uint8_t {
  GazeboSimulation = 0,
  StationWallMonotonic = 1,
};

enum class ClockMapping : std::uint8_t {
  Identity = 0,
  AffineToSession = 1,
};

enum class GuardEventKind : std::uint8_t {
  NewEpoch = 0,
  Locked = 1,
  Degraded = 2,
  Lost = 3,
};

struct ThresholdPolicy {
  std::optional<std::size_t> estimator_window;
  std::uint32_t min_lock_samples{0};
  std::uint32_t recover_lock_samples{0};
  std::uint32_t lost_after_failures{0};
  std::uint64_t startup_lock_timeout_ns{0};
  std::optional<std::uint64_t> max_offset_step_ns;
  std::optional<double> max_drift_ppm;
  std::optional<std::uint64_t> max_jitter_ns;
  std::uint64_t max_uncertainty_ns{0};
  std::uint64_t max_sample_age_ns{0};
  std::uint64_t max_authority_age_ns{0};
  std::optional<std::uint64_t> max_gazebo_clock_skew_ns;
  std::optional<std::uint64_t> max_station_wall_error_ns;
  std::optional<std::uint64_t> max_station_wall_step_ns;
  std::optional<double> min_gazebo_real_time_factor;
  std::optional<double> max_gazebo_real_time_factor;
  std::uint64_t guard_poll_period_ns{0};
  std::uint32_t io_queue_depth{0};
};

struct Route {
  std::string slot;
  std::string source_body;
  std::string canonical_body;
  SourceDomain source_domain{SourceDomain::Simulation};
  std::uint64_t sample_period_ns{0};
  std::uint64_t source_uncertainty_ns{0};
};

struct FrozenConfig {
  std::string schema;
  std::string session_id;
  std::string session_contract_sha256;
  std::string run_mode;
  SessionTimeAuthority session_time_authority{
      SessionTimeAuthority::GazeboSimulation};
  ClockMapping clock_mapping{ClockMapping::Identity};
  std::string policy_sha256;
  std::uint64_t vrpn_wire_resolution_ns{0};
  bool measurement_delay_enabled{false};
  std::string delay_timestamp_policy;
  ThresholdPolicy thresholds;
  std::vector<Route> routes;
};

struct Observation {
  std::string slot;
  std::string source_body;
  SourceDomain source_domain{SourceDomain::Simulation};
  StreamKind stream{StreamKind::Pose};
  std::uint64_t raw_source_stamp_ns{0};
  std::uint64_t receive_monotonic_stamp_ns{0};
  std::uint64_t receive_system_wall_stamp_ns{0};
  // Populated by SessionClockGuard from the frozen authority. Callers must
  // leave these zero.
  std::uint64_t receive_session_stamp_ns{0};
  std::uint64_t gazebo_authority_stamp_ns{0};
  std::uint64_t authority_age_ns{0};
  std::int64_t gazebo_clock_skew_ns{0};
  bool fallback_source{false};
};

struct GazeboClockObservation {
  std::uint64_t gazebo_stamp_ns{0};
  std::uint64_t receive_monotonic_stamp_ns{0};
  std::uint64_t receive_system_wall_stamp_ns{0};
};

struct AuthorityStatus {
  SessionTimeAuthority authority{SessionTimeAuthority::GazeboSimulation};
  ClockMapping mapping{ClockMapping::Identity};
  GuardState state{GuardState::Initializing};
  bool healthy{false};
  bool gazebo_clock_required{false};
  bool gazebo_clock_ready{false};
  std::uint64_t epoch{0};
  std::uint64_t session_now_ns{0};
  std::uint64_t last_authority_monotonic_stamp_ns{0};
  std::uint64_t authority_age_ns{0};
  std::uint64_t last_gazebo_clock_stamp_ns{0};
  std::int64_t last_gazebo_clock_skew_ns{0};
  std::int64_t station_wall_error_ns{0};
  std::int64_t station_wall_step_ns{0};
  double gazebo_real_time_factor{0.0};
  std::uint32_t consecutive_failures{0};
  std::uint32_t recovery_samples{0};
  std::string reason;
};

struct TimestampEnvelope {
  std::string slot;
  std::string source_body;
  std::string canonical_body;
  SourceDomain source_domain{SourceDomain::Simulation};
  StreamKind stream{StreamKind::Pose};
  std::uint64_t epoch{0};
  std::uint64_t raw_source_stamp_ns{0};
  std::uint64_t mapped_session_stamp_ns{0};
  std::uint64_t receive_monotonic_stamp_ns{0};
  std::uint64_t authority_age_ns{0};
  std::int64_t gazebo_clock_skew_ns{0};
  std::int64_t offset_ns{0};
  double drift_ppm{0.0};
  std::uint64_t jitter_ns{0};
  std::uint64_t uncertainty_ns{0};
  GuardState state{GuardState::Initializing};
  bool accepted{false};
  bool canonical_publish_allowed{false};
  std::string reason;
};

struct RouteStatus {
  std::string slot;
  std::string source_body;
  std::string canonical_body;
  SourceDomain source_domain{SourceDomain::Simulation};
  std::uint64_t epoch{0};
  GuardState state{GuardState::Initializing};
  std::int64_t offset_ns{0};
  double drift_ppm{0.0};
  std::uint64_t jitter_ns{0};
  std::uint64_t uncertainty_ns{0};
  std::uint64_t last_raw_source_stamp_ns{0};
  std::uint64_t last_session_stamp_ns{0};
  std::uint64_t last_receive_monotonic_stamp_ns{0};
  std::uint64_t authority_age_ns{0};
  std::int64_t gazebo_clock_skew_ns{0};
  std::int64_t station_wall_error_ns{0};
  std::int64_t station_wall_step_ns{0};
  double gazebo_real_time_factor{0.0};
  std::uint32_t healthy_samples{0};
  std::uint32_t consecutive_failures{0};
  std::string reason;
};

struct GuardEvent {
  std::uint64_t sequence{0};
  GuardEventKind kind{GuardEventKind::NewEpoch};
  GuardState state{GuardState::Initializing};
  std::uint64_t epoch{0};
  std::uint64_t monotonic_stamp_ns{0};
  std::uint64_t session_stamp_ns{0};
  std::uint64_t authority_age_ns{0};
  std::uint64_t last_gazebo_clock_stamp_ns{0};
  std::int64_t gazebo_clock_skew_ns{0};
  std::int64_t station_wall_error_ns{0};
  std::int64_t station_wall_step_ns{0};
  double gazebo_real_time_factor{0.0};
  std::string reason;
};

const char* toString(GuardState state);
const char* toString(SourceDomain domain);
const char* toString(StreamKind stream);
const char* toString(SessionTimeAuthority authority);
const char* toString(ClockMapping mapping);
const char* toString(GuardEventKind kind);
SourceDomain parseSourceDomain(const std::string& text);
SessionTimeAuthority parseSessionTimeAuthority(const std::string& text);
ClockMapping parseClockMapping(const std::string& text);
std::uint64_t parseEpochId(const std::string& text);
std::uint64_t parseRosPrivateEpochId(const std::string& text);

std::string rawRoot(SourceDomain domain);
std::string rawPoseTopic(const Route& route);
std::string rawTwistTopic(const Route& route);
std::string canonicalPoseTopic(const Route& route);
std::string canonicalTwistTopic(const Route& route);
std::string envelopeTopic(const Route& route);
std::string statusTopic(const Route& route);
std::string gazeboClockTopic(const FrozenConfig& config);

}  // namespace xgc_session_clock_guard
