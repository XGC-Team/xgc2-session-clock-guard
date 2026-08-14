#include "xgc_session_clock_guard/session_clock_guard.hpp"

#include "xgc_session_clock_guard/frozen_config.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace xgc_session_clock_guard {
namespace {

std::uint64_t saturatingAdd(std::uint64_t left, std::uint64_t right) {
  if (right > std::numeric_limits<std::uint64_t>::max() - left) {
    return std::numeric_limits<std::uint64_t>::max();
  }
  return left + right;
}

std::int64_t signedDifference(std::uint64_t left, std::uint64_t right) {
  if (left >= right) {
    return static_cast<std::int64_t>(std::min<std::uint64_t>(
        left - right,
        static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())));
  }
  return -static_cast<std::int64_t>(std::min<std::uint64_t>(
      right - left,
      static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())));
}

std::int64_t signedDifference(std::int64_t left, std::int64_t right) {
  const long double difference =
      static_cast<long double>(left) - static_cast<long double>(right);
  if (difference <=
      static_cast<long double>(std::numeric_limits<std::int64_t>::min())) {
    return std::numeric_limits<std::int64_t>::min();
  }
  if (difference >=
      static_cast<long double>(std::numeric_limits<std::int64_t>::max())) {
    return std::numeric_limits<std::int64_t>::max();
  }
  return static_cast<std::int64_t>(difference);
}

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

SessionClockGuard::SessionClockGuard(FrozenConfig config, std::uint64_t epoch,
                                     std::uint64_t monotonic_start_stamp_ns,
                                     std::uint64_t system_wall_start_stamp_ns)
    : config_(std::move(config)) {
  validateThresholdPolicy(config_.thresholds);
  validateRoutes(config_.run_mode, config_.routes);
  validateModeClockPolicy(config_);
  validateSourceTiming(config_);
  for (const auto &route_value : config_.routes) {
    Entry entry;
    entry.route = route_value;
    entry.mapper = std::make_unique<ClockMapper>(
        route_value, config_.thresholds, config_.clock_mapping);
    entries_.emplace(route_value.slot, std::move(entry));
  }
  authority_status_.authority = config_.session_time_authority;
  authority_status_.mapping = config_.clock_mapping;
  authority_status_.state = GuardState::Initializing;
  authority_status_.gazebo_clock_required = config_.run_mode != "physical";
  authority_status_.reason = "validating immutable startup epoch";

  if (epoch == 0U || epoch <= epoch_ || monotonic_start_stamp_ns == 0U ||
      system_wall_start_stamp_ns == 0U) {
    throw std::invalid_argument("startup epoch and both steady and system-wall "
                                "anchors must be nonzero");
  }

  const std::uint64_t session_start_stamp_ns =
      config_.session_time_authority ==
              SessionTimeAuthority::StationWallMonotonic
          ? system_wall_start_stamp_ns
          : 0U;
  for (auto &item : entries_) {
    std::string route_reason;
    if (!item.second.mapper->initializeEpoch(epoch, session_start_stamp_ns,
                                             monotonic_start_stamp_ns,
                                             &route_reason)) {
      throw std::invalid_argument("route " + item.first +
                                  " rejected startup epoch: " + route_reason);
    }
  }

  epoch_ = epoch;
  epoch_start_monotonic_stamp_ns_ = monotonic_start_stamp_ns;
  epoch_wall_anchor_ns_ = system_wall_start_stamp_ns;
  station_healthy_ = config_.session_time_authority ==
                     SessionTimeAuthority::StationWallMonotonic;
  gazebo_healthy_ = false;
  have_gazebo_clock_ = false;
  have_gazebo_rate_ = false;
  last_gazebo_clock_stamp_ns_ = 0U;
  last_gazebo_receive_monotonic_stamp_ns_ = 0U;
  last_station_wall_error_ns_ = 0;
  authority_state_ = config_.run_mode == "physical" ? GuardState::Locked
                                                    : GuardState::Initializing;
  authority_consecutive_failures_ = 0U;
  authority_recovery_samples_ = 0U;

  authority_status_ = {};
  authority_status_.authority = config_.session_time_authority;
  authority_status_.mapping = config_.clock_mapping;
  authority_status_.state = authority_state_;
  authority_status_.gazebo_clock_required = config_.run_mode != "physical";
  authority_status_.epoch = epoch_;
  authority_status_.session_now_ns = session_start_stamp_ns;
  authority_status_.last_authority_monotonic_stamp_ns =
      config_.run_mode == "physical" ? monotonic_start_stamp_ns : 0U;
  authority_status_.reason =
      config_.run_mode == "physical"
          ? (entries_.empty()
                 ? "station wall anchor accepted; authority-only Session is "
                   "locked"
                 : "station wall anchor accepted; routes are initializing")
          : "waiting for a positive admitted Gazebo clock";
  refreshAuthorityHealth();

  const std::string accepted_reason =
      entries_.empty()
          ? "Core-started authority-only epoch accepted; anchors are immutable"
          : "Core-started epoch accepted; anchors are immutable and every "
            "route is initializing";
  emitEvent(GuardEventKind::NewEpoch, GuardState::Initializing, accepted_reason,
            monotonic_start_stamp_ns);
  if (aggregateState() == GuardState::Locked) {
    emitEvent(GuardEventKind::Locked, GuardState::Locked,
              authority_status_.reason, monotonic_start_stamp_ns);
  }
}

std::uint64_t
SessionClockGuard::sessionNow(std::uint64_t monotonic_now_ns) const {
  if (epoch_ == 0U) {
    return 0U;
  }
  if (config_.session_time_authority ==
      SessionTimeAuthority::GazeboSimulation) {
    return last_gazebo_clock_stamp_ns_;
  }
  if (monotonic_now_ns < epoch_start_monotonic_stamp_ns_) {
    return 0U;
  }
  return saturatingAdd(epoch_wall_anchor_ns_,
                       monotonic_now_ns - epoch_start_monotonic_stamp_ns_);
}

void SessionClockGuard::refreshAuthorityHealth() {
  authority_status_.gazebo_clock_ready =
      have_gazebo_clock_ && (config_.run_mode != "hybrid" || have_gazebo_rate_);
  const bool station_required = config_.session_time_authority ==
                                SessionTimeAuthority::StationWallMonotonic;
  authority_status_.state = authority_state_;
  authority_status_.consecutive_failures = authority_consecutive_failures_;
  authority_status_.recovery_samples = authority_recovery_samples_;
  authority_status_.healthy =
      authority_state_ == GuardState::Locked &&
      (!station_required || station_healthy_) &&
      (!authority_status_.gazebo_clock_required ||
       (gazebo_healthy_ && authority_status_.gazebo_clock_ready));
}

bool SessionClockGuard::validateStationClock(std::uint64_t monotonic_now_ns,
                                             std::uint64_t system_wall_now_ns,
                                             std::string *reason) {
  if (monotonic_now_ns == 0U || system_wall_now_ns == 0U ||
      monotonic_now_ns < epoch_start_monotonic_stamp_ns_) {
    station_healthy_ = false;
    refreshAuthorityHealth();
    if (reason != nullptr) {
      *reason =
          "station clock observation has an invalid or backwards steady stamp";
    }
    return false;
  }
  const std::uint64_t derived_session_now = sessionNow(monotonic_now_ns);
  const std::int64_t wall_error =
      signedDifference(system_wall_now_ns, derived_session_now);
  const std::int64_t wall_step =
      signedDifference(wall_error, last_station_wall_error_ns_);
  last_station_wall_error_ns_ = wall_error;
  authority_status_.session_now_ns = derived_session_now;
  authority_status_.last_authority_monotonic_stamp_ns = monotonic_now_ns;
  authority_status_.authority_age_ns = 0U;
  authority_status_.station_wall_error_ns = wall_error;
  authority_status_.station_wall_step_ns = wall_step;

  if (absoluteUnsigned(wall_error) >
      *config_.thresholds.max_station_wall_error_ns) {
    station_healthy_ = false;
    refreshAuthorityHealth();
    if (reason != nullptr) {
      *reason = "station system wall diverged from the immutable wall-at-epoch "
                "plus steady timeline";
    }
    return false;
  }
  if (absoluteUnsigned(wall_step) >
      *config_.thresholds.max_station_wall_step_ns) {
    station_healthy_ = false;
    refreshAuthorityHealth();
    if (reason != nullptr) {
      *reason = "station system wall step exceeds the frozen threshold";
    }
    return false;
  }
  station_healthy_ = true;
  refreshAuthorityHealth();
  if (config_.run_mode == "physical") {
    authority_status_.reason =
        "station wall-at-epoch plus steady authority is healthy";
  }
  if (reason != nullptr) {
    *reason = "station wall-at-epoch plus steady authority is healthy";
  }
  return true;
}

bool SessionClockGuard::validateGazeboAuthorityAge(
    std::uint64_t monotonic_now_ns, std::string *reason, bool *immediate_loss) {
  if (immediate_loss != nullptr) {
    *immediate_loss = false;
  }
  if (!authority_status_.gazebo_clock_required) {
    return true;
  }
  if (monotonic_now_ns == 0U ||
      monotonic_now_ns < epoch_start_monotonic_stamp_ns_ ||
      (have_gazebo_clock_ &&
       monotonic_now_ns < last_gazebo_receive_monotonic_stamp_ns_)) {
    if (immediate_loss != nullptr) {
      *immediate_loss = true;
    }
    if (reason != nullptr) {
      *reason =
          "steady clock moved backwards while checking Gazebo authority age";
    }
    return false;
  }
  if (authority_state_ == GuardState::Lost) {
    if (reason != nullptr) {
      *reason = "clock authority is lost; a strictly newer epoch is required";
    }
    return false;
  }
  const std::uint64_t reference = have_gazebo_clock_
                                      ? last_gazebo_receive_monotonic_stamp_ns_
                                      : epoch_start_monotonic_stamp_ns_;
  authority_status_.authority_age_ns = monotonic_now_ns - reference;
  if (!have_gazebo_clock_) {
    const bool startup_timed_out = authority_status_.authority_age_ns >
                                   config_.thresholds.startup_lock_timeout_ns;
    if (startup_timed_out && immediate_loss != nullptr) {
      *immediate_loss = true;
    }
    if (reason != nullptr) {
      *reason = startup_timed_out
                    ? "positive Gazebo clock did not arrive before the frozen "
                      "startup lock timeout"
                    : "waiting for a positive admitted Gazebo clock";
    }
    return false;
  }
  if (authority_status_.authority_age_ns >
      config_.thresholds.max_authority_age_ns) {
    if (reason != nullptr) {
      *reason = "Gazebo clock authority is stale by steady-clock age";
    }
    return false;
  }
  if (config_.run_mode == "hybrid" && !have_gazebo_rate_) {
    if (reason != nullptr) {
      *reason = "waiting for a second private Gazebo clock sample to establish "
                "real-time factor";
    }
    return false;
  }
  return true;
}

void SessionClockGuard::failAuthority(const std::string &reason,
                                      bool immediate_loss) {
  if (config_.session_time_authority ==
      SessionTimeAuthority::StationWallMonotonic) {
    station_healthy_ = false;
  }
  if (authority_status_.gazebo_clock_required) {
    gazebo_healthy_ = false;
  }
  if (authority_state_ != GuardState::Lost) {
    if (authority_consecutive_failures_ <
        std::numeric_limits<std::uint32_t>::max()) {
      ++authority_consecutive_failures_;
    }
    authority_recovery_samples_ = 0U;
    authority_state_ =
        immediate_loss || authority_consecutive_failures_ >=
                              config_.thresholds.lost_after_failures
            ? GuardState::Lost
            : GuardState::Degraded;
  }
  authority_status_.reason = reason;
  refreshAuthorityHealth();
  for (auto &item : entries_) {
    item.second.mapper->authorityFailure(reason, immediate_loss);
  }
}

void SessionClockGuard::recordHealthyAuthority(const std::string &reason) {
  if (authority_state_ == GuardState::Lost) {
    authority_status_.reason =
        reason + "; authority is lost and a strictly newer epoch is required";
    refreshAuthorityHealth();
    return;
  }
  authority_consecutive_failures_ = 0U;
  if (authority_state_ == GuardState::Initializing) {
    authority_state_ = GuardState::Locked;
    authority_recovery_samples_ = 0U;
  } else if (authority_state_ == GuardState::Degraded) {
    if (authority_recovery_samples_ <
        std::numeric_limits<std::uint32_t>::max()) {
      ++authority_recovery_samples_;
    }
    if (authority_recovery_samples_ >=
        config_.thresholds.recover_lock_samples) {
      authority_state_ = GuardState::Locked;
      authority_recovery_samples_ = 0U;
    }
  }
  authority_status_.reason = reason;
  refreshAuthorityHealth();
}

bool SessionClockGuard::observeGazeboClock(
    const GazeboClockObservation &observation, std::string *reason) {
  const GuardState before = aggregateState();
  if (epoch_ == 0U) {
    if (reason != nullptr) {
      *reason = "epoch has not been started";
    }
    return false;
  }
  if (!authority_status_.gazebo_clock_required) {
    if (reason != nullptr) {
      *reason = "physical mode forbids a Gazebo clock authority input";
    }
    return false;
  }
  if (config_.run_mode == "hybrid") {
    std::string station_reason;
    if (!validateStationClock(observation.receive_monotonic_stamp_ns,
                              observation.receive_system_wall_stamp_ns,
                              &station_reason)) {
      failAuthority(station_reason, true);
      emitStateTransition(before, station_reason,
                          observation.receive_monotonic_stamp_ns);
      if (reason != nullptr) {
        *reason = station_reason;
      }
      return false;
    }
  }
  if (observation.receive_monotonic_stamp_ns == 0U ||
      observation.receive_monotonic_stamp_ns <
          epoch_start_monotonic_stamp_ns_ ||
      (have_gazebo_clock_ && observation.receive_monotonic_stamp_ns <=
                                 last_gazebo_receive_monotonic_stamp_ns_)) {
    const std::string failure =
        "Gazebo clock receipt has a repeated or backwards steady timestamp";
    failAuthority(failure, true);
    emitStateTransition(before, failure,
                        observation.receive_monotonic_stamp_ns);
    if (reason != nullptr) {
      *reason = failure;
    }
    return false;
  }
  if (!have_gazebo_clock_ &&
      observation.receive_monotonic_stamp_ns - epoch_start_monotonic_stamp_ns_ >
          config_.thresholds.startup_lock_timeout_ns) {
    const std::string failure = "positive Gazebo clock did not arrive before "
                                "the frozen startup lock timeout";
    failAuthority(failure, true);
    emitStateTransition(before, failure,
                        observation.receive_monotonic_stamp_ns);
    if (reason != nullptr) {
      *reason = failure;
    }
    return false;
  }
  if (observation.gazebo_stamp_ns == 0U) {
    if (!have_gazebo_clock_) {
      authority_status_.reason = "waiting for the first positive Gazebo clock";
      refreshAuthorityHealth();
      if (reason != nullptr) {
        *reason = authority_status_.reason;
      }
      return false;
    }
    const std::string failure =
        "Gazebo clock returned to zero after stream start";
    failAuthority(failure, true);
    emitStateTransition(before, failure,
                        observation.receive_monotonic_stamp_ns);
    if (reason != nullptr) {
      *reason = failure;
    }
    return false;
  }

  double next_real_time_factor = 0.0;
  if (have_gazebo_clock_) {
    if (observation.gazebo_stamp_ns < last_gazebo_clock_stamp_ns_) {
      const std::string failure = "Gazebo clock moved backwards";
      failAuthority(failure, true);
      emitStateTransition(before, failure,
                          observation.receive_monotonic_stamp_ns);
      if (reason != nullptr) {
        *reason = failure;
      }
      return false;
    }
    if (observation.gazebo_stamp_ns == last_gazebo_clock_stamp_ns_) {
      const std::string failure = "Gazebo clock stagnated";
      failAuthority(failure, false);
      emitStateTransition(before, failure,
                          observation.receive_monotonic_stamp_ns);
      if (reason != nullptr) {
        *reason = failure;
      }
      return false;
    }
    if (config_.run_mode == "hybrid") {
      const std::uint64_t gazebo_delta =
          observation.gazebo_stamp_ns - last_gazebo_clock_stamp_ns_;
      const std::uint64_t steady_delta =
          observation.receive_monotonic_stamp_ns -
          last_gazebo_receive_monotonic_stamp_ns_;
      next_real_time_factor =
          static_cast<double>(gazebo_delta) / static_cast<double>(steady_delta);
      authority_status_.gazebo_real_time_factor = next_real_time_factor;
      if (!std::isfinite(next_real_time_factor) ||
          next_real_time_factor <
              *config_.thresholds.min_gazebo_real_time_factor ||
          next_real_time_factor >
              *config_.thresholds.max_gazebo_real_time_factor) {
        const std::string failure = "private Gazebo clock real-time factor "
                                    "exceeds the frozen Hybrid bounds";
        failAuthority(failure, false);
        emitStateTransition(before, failure,
                            observation.receive_monotonic_stamp_ns);
        if (reason != nullptr) {
          *reason = failure;
        }
        return false;
      }
      have_gazebo_rate_ = true;
    }
  }

  have_gazebo_clock_ = true;
  last_gazebo_clock_stamp_ns_ = observation.gazebo_stamp_ns;
  last_gazebo_receive_monotonic_stamp_ns_ =
      observation.receive_monotonic_stamp_ns;
  gazebo_healthy_ = config_.run_mode == "simulation" || have_gazebo_rate_;
  authority_status_.last_gazebo_clock_stamp_ns = observation.gazebo_stamp_ns;
  authority_status_.last_authority_monotonic_stamp_ns =
      observation.receive_monotonic_stamp_ns;
  authority_status_.authority_age_ns = 0U;
  authority_status_.session_now_ns =
      config_.run_mode == "simulation"
          ? observation.gazebo_stamp_ns
          : sessionNow(observation.receive_monotonic_stamp_ns);
  authority_status_.gazebo_real_time_factor = next_real_time_factor;
  authority_status_.reason =
      config_.run_mode == "hybrid" && !have_gazebo_rate_
          ? "first private Gazebo clock accepted; waiting for real-time-factor "
            "sample"
          : "Gazebo clock authority is positive, monotonic, and fresh";
  if (gazebo_healthy_) {
    recordHealthyAuthority(authority_status_.reason);
  } else {
    refreshAuthorityHealth();
  }
  emitStateTransition(before, authority_status_.reason,
                      observation.receive_monotonic_stamp_ns);
  if (reason != nullptr) {
    *reason = authority_status_.reason;
  }
  return true;
}

TimestampEnvelope
SessionClockGuard::rejectForAuthority(const Observation &observation,
                                      const std::string &reason) {
  const auto it = entries_.find(observation.slot);
  auto enriched = observation;
  enriched.authority_age_ns = authority_status_.authority_age_ns;
  enriched.gazebo_authority_stamp_ns = last_gazebo_clock_stamp_ns_;
  enriched.gazebo_clock_skew_ns = authority_status_.last_gazebo_clock_skew_ns;
  return it->second.mapper->rejectedEnvelope(enriched, reason);
}

TimestampEnvelope SessionClockGuard::observe(const Observation &observation) {
  const auto it = entries_.find(observation.slot);
  if (it == entries_.end()) {
    TimestampEnvelope envelope;
    envelope.slot = observation.slot;
    envelope.source_body = observation.source_body;
    envelope.source_domain = observation.source_domain;
    envelope.stream = observation.stream;
    envelope.epoch = epoch_;
    envelope.raw_source_stamp_ns = observation.raw_source_stamp_ns;
    envelope.receive_monotonic_stamp_ns =
        observation.receive_monotonic_stamp_ns;
    envelope.state = GuardState::Lost;
    envelope.reason = "slot is not present in the frozen route table";
    return envelope;
  }

  const GuardState before = aggregateState();
  if (epoch_ == 0U) {
    return rejectForAuthority(observation, "epoch has not been started");
  }

  auto enriched = observation;
  enriched.receive_session_stamp_ns = 0U;
  enriched.gazebo_authority_stamp_ns = 0U;
  enriched.authority_age_ns = 0U;
  enriched.gazebo_clock_skew_ns = 0;

  if (config_.session_time_authority ==
      SessionTimeAuthority::StationWallMonotonic) {
    std::string station_reason;
    if (!validateStationClock(observation.receive_monotonic_stamp_ns,
                              observation.receive_system_wall_stamp_ns,
                              &station_reason)) {
      failAuthority(station_reason, true);
      emitStateTransition(before, station_reason,
                          observation.receive_monotonic_stamp_ns);
      return rejectForAuthority(enriched, station_reason);
    }
    enriched.receive_session_stamp_ns =
        sessionNow(observation.receive_monotonic_stamp_ns);
  }

  if (authority_status_.gazebo_clock_required) {
    std::string age_reason;
    bool immediate_loss = false;
    if (!validateGazeboAuthorityAge(observation.receive_monotonic_stamp_ns,
                                    &age_reason, &immediate_loss)) {
      const bool deadline_exceeded =
          have_gazebo_clock_ && authority_status_.authority_age_ns >
                                    config_.thresholds.max_authority_age_ns;
      if (immediate_loss || deadline_exceeded) {
        failAuthority(age_reason, immediate_loss);
        emitStateTransition(before, age_reason,
                            observation.receive_monotonic_stamp_ns);
      }
      return rejectForAuthority(enriched, age_reason);
    }
    if (!authority_status_.healthy) {
      return rejectForAuthority(enriched, authority_status_.reason);
    }
    enriched.gazebo_authority_stamp_ns = last_gazebo_clock_stamp_ns_;
    enriched.authority_age_ns = authority_status_.authority_age_ns;
    if (config_.run_mode == "simulation") {
      enriched.receive_session_stamp_ns = last_gazebo_clock_stamp_ns_;
    }
    if (it->second.route.source_domain == SourceDomain::Simulation) {
      enriched.gazebo_clock_skew_ns = signedDifference(
          observation.raw_source_stamp_ns, last_gazebo_clock_stamp_ns_);
      authority_status_.last_gazebo_clock_skew_ns =
          enriched.gazebo_clock_skew_ns;
      if (absoluteUnsigned(enriched.gazebo_clock_skew_ns) >
          *config_.thresholds.max_gazebo_clock_skew_ns) {
        const std::string failure =
            "simulation VRPN source stamp exceeds the frozen Gazebo clock skew";
        it->second.mapper->authorityFailure(failure, false);
        emitStateTransition(before, failure,
                            observation.receive_monotonic_stamp_ns);
        return it->second.mapper->rejectedEnvelope(enriched, failure);
      }
    }
  }

  auto envelope = it->second.mapper->observe(enriched);
  envelope.canonical_publish_allowed =
      envelope.accepted && sensitiveOutputAllowed();
  emitStateTransition(before, envelope.reason,
                      observation.receive_monotonic_stamp_ns);
  return envelope;
}

void SessionClockGuard::poll(std::uint64_t monotonic_now_ns,
                             std::uint64_t system_wall_now_ns) {
  if (epoch_ == 0U) {
    return;
  }
  const GuardState before = aggregateState();
  if (config_.session_time_authority ==
      SessionTimeAuthority::StationWallMonotonic) {
    std::string station_reason;
    if (!validateStationClock(monotonic_now_ns, system_wall_now_ns,
                              &station_reason)) {
      failAuthority(station_reason, true);
      emitStateTransition(before, station_reason, monotonic_now_ns);
      return;
    }
  }
  if (authority_status_.gazebo_clock_required) {
    std::string age_reason;
    bool immediate_loss = false;
    if (!validateGazeboAuthorityAge(monotonic_now_ns, &age_reason,
                                    &immediate_loss)) {
      const bool deadline_exceeded =
          have_gazebo_clock_ && authority_status_.authority_age_ns >
                                    config_.thresholds.max_authority_age_ns;
      if (immediate_loss || deadline_exceeded) {
        failAuthority(age_reason, immediate_loss);
        emitStateTransition(before, age_reason, monotonic_now_ns);
      }
      return;
    }
  }
  for (auto &item : entries_) {
    item.second.mapper->poll(monotonic_now_ns);
  }
  authority_status_.session_now_ns = sessionNow(monotonic_now_ns);
  emitStateTransition(before, "steady-clock freshness poll completed",
                      monotonic_now_ns);
}

RouteStatus SessionClockGuard::routeStatus(const std::string &slot) const {
  const auto it = entries_.find(slot);
  if (it == entries_.end()) {
    throw std::out_of_range("unknown frozen route slot: " + slot);
  }
  RouteStatus status = it->second.mapper->status();
  status.authority_age_ns = authority_status_.authority_age_ns;
  status.station_wall_error_ns = authority_status_.station_wall_error_ns;
  status.station_wall_step_ns = authority_status_.station_wall_step_ns;
  status.gazebo_real_time_factor = authority_status_.gazebo_real_time_factor;
  return status;
}

std::vector<RouteStatus> SessionClockGuard::routeStatuses() const {
  std::vector<RouteStatus> statuses;
  statuses.reserve(entries_.size());
  for (const auto &item : entries_) {
    statuses.push_back(routeStatus(item.first));
  }
  return statuses;
}

GuardState SessionClockGuard::aggregateState() const {
  bool initializing = false;
  bool degraded = false;
  for (const auto &item : entries_) {
    switch (item.second.mapper->status().state) {
    case GuardState::Lost:
      return GuardState::Lost;
    case GuardState::Degraded:
      degraded = true;
      break;
    case GuardState::Initializing:
      initializing = true;
      break;
    case GuardState::Locked:
      break;
    }
  }
  if (authority_state_ == GuardState::Lost) {
    return GuardState::Lost;
  }
  if (degraded || authority_state_ == GuardState::Degraded) {
    return GuardState::Degraded;
  }
  if (initializing || authority_state_ == GuardState::Initializing ||
      !authority_status_.healthy) {
    return GuardState::Initializing;
  }
  return GuardState::Locked;
}

bool SessionClockGuard::sensitiveOutputAllowed() const {
  return authority_status_.healthy && aggregateState() == GuardState::Locked;
}

std::size_t SessionClockGuard::lockedRouteCount() const {
  std::size_t count = 0U;
  for (const auto &item : entries_) {
    if (item.second.mapper->status().state == GuardState::Locked) {
      ++count;
    }
  }
  return count;
}

void SessionClockGuard::emitStateTransition(GuardState before,
                                            const std::string &reason,
                                            std::uint64_t monotonic_stamp_ns) {
  const GuardState after = aggregateState();
  if (after == before) {
    return;
  }
  switch (after) {
  case GuardState::Locked:
    emitEvent(GuardEventKind::Locked, after, reason, monotonic_stamp_ns);
    break;
  case GuardState::Degraded:
    emitEvent(GuardEventKind::Degraded, after, reason, monotonic_stamp_ns);
    break;
  case GuardState::Lost:
    emitEvent(GuardEventKind::Lost, after, reason, monotonic_stamp_ns);
    break;
  case GuardState::Initializing:
    break;
  }
}

void SessionClockGuard::emitEvent(GuardEventKind kind, GuardState state,
                                  const std::string &reason,
                                  std::uint64_t monotonic_stamp_ns) {
  GuardEvent event;
  event.sequence = next_event_sequence_++;
  event.kind = kind;
  event.state = state;
  event.epoch = epoch_;
  event.monotonic_stamp_ns = monotonic_stamp_ns;
  event.session_stamp_ns = sessionNow(monotonic_stamp_ns);
  event.authority_age_ns = authority_status_.authority_age_ns;
  event.last_gazebo_clock_stamp_ns =
      authority_status_.last_gazebo_clock_stamp_ns;
  event.gazebo_clock_skew_ns = authority_status_.last_gazebo_clock_skew_ns;
  event.station_wall_error_ns = authority_status_.station_wall_error_ns;
  event.station_wall_step_ns = authority_status_.station_wall_step_ns;
  event.gazebo_real_time_factor = authority_status_.gazebo_real_time_factor;
  event.reason = reason;
  pending_events_.push_back(std::move(event));
}

std::vector<GuardEvent> SessionClockGuard::takeEvents() {
  std::vector<GuardEvent> events;
  events.swap(pending_events_);
  return events;
}

void SessionClockGuard::failEpochFence(const std::string &reason,
                                       std::uint64_t monotonic_stamp_ns) {
  const GuardState before = aggregateState();
  failAuthority(reason, true);
  emitStateTransition(before, reason, monotonic_stamp_ns);
}

const Route &SessionClockGuard::route(const std::string &slot) const {
  const auto it = entries_.find(slot);
  if (it == entries_.end()) {
    throw std::out_of_range("unknown frozen route slot: " + slot);
  }
  return it->second.route;
}

} // namespace xgc_session_clock_guard
