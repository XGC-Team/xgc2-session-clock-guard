#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "xgc_session_clock_guard/clock_mapper.hpp"
#include "xgc_session_clock_guard/types.hpp"

namespace xgc_session_clock_guard {

class SessionClockGuard {
 public:
  SessionClockGuard(FrozenConfig config,
                    std::uint64_t epoch,
                    std::uint64_t monotonic_start_stamp_ns,
                    std::uint64_t system_wall_start_stamp_ns);

  bool observeGazeboClock(const GazeboClockObservation& observation,
                          std::string* reason);
  TimestampEnvelope observe(const Observation& observation);
  void poll(std::uint64_t monotonic_now_ns,
            std::uint64_t system_wall_now_ns);
  RouteStatus routeStatus(const std::string& slot) const;
  std::vector<RouteStatus> routeStatuses() const;
  AuthorityStatus authorityStatus() const { return authority_status_; }
  GuardState aggregateState() const;
  bool sensitiveOutputAllowed() const;
  std::size_t lockedRouteCount() const;
  std::uint64_t epoch() const { return epoch_; }
  std::uint64_t sessionNow(std::uint64_t monotonic_now_ns) const;
  std::vector<GuardEvent> takeEvents();
  void failEpochFence(const std::string& reason,
                      std::uint64_t monotonic_stamp_ns);
  const FrozenConfig& config() const { return config_; }
  const Route& route(const std::string& slot) const;

 private:
  struct Entry {
    Route route;
    std::unique_ptr<ClockMapper> mapper;
  };

  bool validateStationClock(std::uint64_t monotonic_now_ns,
                            std::uint64_t system_wall_now_ns,
                            std::string* reason);
  bool validateGazeboAuthorityAge(std::uint64_t monotonic_now_ns,
                                  std::string* reason,
                                  bool* immediate_loss);
  void failAuthority(const std::string& reason, bool immediate_loss);
  void recordHealthyAuthority(const std::string& reason);
  void refreshAuthorityHealth();
  void emitStateTransition(GuardState before,
                           const std::string& reason,
                           std::uint64_t monotonic_stamp_ns);
  void emitEvent(GuardEventKind kind,
                 GuardState state,
                 const std::string& reason,
                 std::uint64_t monotonic_stamp_ns);
  TimestampEnvelope rejectForAuthority(const Observation& observation,
                                       const std::string& reason);

  FrozenConfig config_;
  std::map<std::string, Entry> entries_;
  std::uint64_t epoch_{0};
  std::uint64_t epoch_start_monotonic_stamp_ns_{0};
  std::uint64_t epoch_wall_anchor_ns_{0};
  bool station_healthy_{false};
  bool gazebo_healthy_{false};
  bool have_gazebo_clock_{false};
  bool have_gazebo_rate_{false};
  std::uint64_t last_gazebo_clock_stamp_ns_{0};
  std::uint64_t last_gazebo_receive_monotonic_stamp_ns_{0};
  std::int64_t last_station_wall_error_ns_{0};
  GuardState authority_state_{GuardState::Initializing};
  std::uint32_t authority_consecutive_failures_{0};
  std::uint32_t authority_recovery_samples_{0};
  AuthorityStatus authority_status_;
  std::uint64_t next_event_sequence_{1};
  std::vector<GuardEvent> pending_events_;
};

}  // namespace xgc_session_clock_guard
