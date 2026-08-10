#include "xgc_session_clock_guard/clock_mapper.hpp"

#include "xgc_session_clock_guard/frozen_config.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace xgc_session_clock_guard {
namespace {

std::uint64_t absoluteUnsigned(std::int64_t value) {
  if (value >= 0) {
    return static_cast<std::uint64_t>(value);
  }
  if (value == std::numeric_limits<std::int64_t>::min()) {
    return static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()) + 1U;
  }
  return static_cast<std::uint64_t>(-value);
}

std::uint64_t saturatingAdd(std::uint64_t left, std::uint64_t right) {
  if (right > std::numeric_limits<std::uint64_t>::max() - left) {
    return std::numeric_limits<std::uint64_t>::max();
  }
  return left + right;
}

std::uint64_t roundedUnsigned(long double value) {
  if (!std::isfinite(static_cast<double>(value)) || value <= 0.0L) {
    return 0U;
  }
  if (value >= static_cast<long double>(std::numeric_limits<std::uint64_t>::max())) {
    return std::numeric_limits<std::uint64_t>::max();
  }
  return static_cast<std::uint64_t>(std::llround(value));
}

std::uint64_t absoluteDifference(std::int64_t left, std::int64_t right) {
  const long double difference = std::fabs(static_cast<long double>(left) -
                                           static_cast<long double>(right));
  return roundedUnsigned(difference);
}

std::int64_t roundedSigned(long double value) {
  if (!std::isfinite(static_cast<double>(value))) {
    return value < 0.0L ? std::numeric_limits<std::int64_t>::min()
                        : std::numeric_limits<std::int64_t>::max();
  }
  if (value <= static_cast<long double>(std::numeric_limits<std::int64_t>::min())) {
    return std::numeric_limits<std::int64_t>::min();
  }
  if (value >= static_cast<long double>(std::numeric_limits<std::int64_t>::max())) {
    return std::numeric_limits<std::int64_t>::max();
  }
  return static_cast<std::int64_t>(std::llround(value));
}

std::size_t streamIndex(StreamKind stream) {
  switch (stream) {
    case StreamKind::Pose:
      return 0U;
    case StreamKind::Twist:
      return 1U;
  }
  throw std::invalid_argument("unknown stream kind");
}

}  // namespace

ClockMapper::ClockMapper(const Route& route,
                         const ThresholdPolicy& policy,
                         ClockMapping mapping)
    : route_(route), policy_(policy), mapping_(mapping) {
  validateThresholdPolicy(policy_);
  status_.slot = route_.slot;
  status_.source_body = route_.source_body;
  status_.canonical_body = route_.canonical_body;
  status_.source_domain = route_.source_domain;
  status_.reason = "epoch has not been started";
}

bool ClockMapper::initializeEpoch(std::uint64_t epoch,
                                  std::uint64_t session_start_stamp_ns,
                                  std::uint64_t monotonic_start_stamp_ns,
                                  std::string* reason) {
  const bool invalid_session_start =
      mapping_ == ClockMapping::AffineToSession && session_start_stamp_ns == 0U;
  if (epoch == 0U || epoch <= status_.epoch || invalid_session_start ||
      monotonic_start_stamp_ns == 0U) {
    if (reason != nullptr) {
      *reason =
          "epoch must be strictly newer, monotonic start must be nonzero, and affine Session start must be nonzero";
    }
    return false;
  }
  samples_.clear();
  timelines_ = {};
  have_estimate_ = false;
  epoch_start_session_stamp_ns_ = session_start_stamp_ns;
  epoch_start_monotonic_stamp_ns_ = monotonic_start_stamp_ns;
  last_counted_healthy_raw_ns_ = 0U;
  recovery_samples_ = 0U;
  status_.epoch = epoch;
  status_.state = GuardState::Initializing;
  status_.offset_ns = 0;
  status_.drift_ppm = 0.0;
  status_.jitter_ns = 0U;
  status_.uncertainty_ns = route_.source_uncertainty_ns;
  status_.last_raw_source_stamp_ns = 0U;
  status_.last_session_stamp_ns = 0U;
  status_.last_receive_monotonic_stamp_ns = 0U;
  status_.authority_age_ns = 0U;
  status_.gazebo_clock_skew_ns = 0;
  status_.station_wall_error_ns = 0;
  status_.station_wall_step_ns = 0;
  status_.gazebo_real_time_factor = 0.0;
  status_.healthy_samples = 0U;
  status_.consecutive_failures = 0U;
  status_.reason = mapping_ == ClockMapping::Identity
                       ? "collecting identity-mapped source samples"
                       : "collecting source-to-Session affine samples";
  if (reason != nullptr) {
    *reason = status_.reason;
  }
  return true;
}

void ClockMapper::poll(std::uint64_t monotonic_now_ns) {
  if (status_.epoch == 0U || status_.state == GuardState::Lost) {
    return;
  }
  if (monotonic_now_ns < epoch_start_monotonic_stamp_ns_) {
    recordFailure("steady clock moved before the epoch start", true);
    return;
  }
  for (const auto& timeline : timelines_) {
    const std::uint64_t reference =
        timeline.seen ? timeline.monotonic_ns : epoch_start_monotonic_stamp_ns_;
    if (monotonic_now_ns < reference) {
      recordFailure("steady clock moved backwards", true);
      return;
    }
    const std::uint64_t age_limit =
        timeline.seen ? policy_.max_sample_age_ns
                      : policy_.startup_lock_timeout_ns;
    if (monotonic_now_ns - reference > age_limit) {
      recordFailure(
          timeline.seen
              ? "VRPN stream silence exceeds frozen maximum age"
              : "required VRPN stream did not start before the frozen startup lock timeout",
          !timeline.seen);
      return;
    }
  }
}

ClockMapper::Estimate ClockMapper::estimateAffine(const std::deque<Sample>& samples,
                                                  std::uint64_t raw_ns) const {
  Estimate result;
  if (samples.empty()) {
    return result;
  }

  const long double raw_anchor = static_cast<long double>(samples.front().raw_ns);
  long double mean_x = 0.0L;
  long double mean_y = 0.0L;
  for (const auto& sample : samples) {
    mean_x += static_cast<long double>(sample.raw_ns) - raw_anchor;
    mean_y += static_cast<long double>(sample.session_ns) -
              static_cast<long double>(sample.raw_ns);
  }
  mean_x /= static_cast<long double>(samples.size());
  mean_y /= static_cast<long double>(samples.size());

  long double covariance = 0.0L;
  long double variance = 0.0L;
  for (const auto& sample : samples) {
    const long double x = static_cast<long double>(sample.raw_ns) - raw_anchor;
    const long double y = static_cast<long double>(sample.session_ns) -
                          static_cast<long double>(sample.raw_ns);
    covariance += (x - mean_x) * (y - mean_y);
    variance += (x - mean_x) * (x - mean_x);
  }
  const long double slope = variance > 0.0L ? covariance / variance : 0.0L;
  const long double intercept = mean_y - slope * mean_x;
  const long double current_x = static_cast<long double>(raw_ns) - raw_anchor;
  const long double current_offset = intercept + slope * current_x;

  long double max_absolute_residual = 0.0L;
  for (const auto& sample : samples) {
    const long double x = static_cast<long double>(sample.raw_ns) - raw_anchor;
    const long double y = static_cast<long double>(sample.session_ns) -
                          static_cast<long double>(sample.raw_ns);
    const long double residual = std::fabs(y - (intercept + slope * x));
    max_absolute_residual = std::max(max_absolute_residual, residual);
  }

  result.offset_ns = roundedSigned(current_offset);
  result.drift_ppm = static_cast<double>(slope * 1000000.0L);
  result.jitter_ns = roundedUnsigned(max_absolute_residual);
  result.uncertainty_ns = saturatingAdd(route_.source_uncertainty_ns, result.jitter_ns);
  const long double mapped = static_cast<long double>(raw_ns) + current_offset;
  result.mapped_session_ns = roundedUnsigned(mapped);
  return result;
}

ClockMapper::Estimate ClockMapper::estimateIdentity(std::uint64_t raw_ns) const {
  Estimate result;
  result.mapped_session_ns = raw_ns;
  result.uncertainty_ns = route_.source_uncertainty_ns;
  return result;
}

void ClockMapper::recordFailure(const std::string& reason, bool immediate_loss) {
  if (status_.state == GuardState::Lost) {
    return;
  }
  if (status_.consecutive_failures < std::numeric_limits<std::uint32_t>::max()) {
    ++status_.consecutive_failures;
  }
  status_.healthy_samples = 0U;
  recovery_samples_ = 0U;
  if (immediate_loss || status_.consecutive_failures >= policy_.lost_after_failures) {
    status_.state = GuardState::Lost;
    status_.reason =
        reason + "; a new Core-started process with a strictly newer epoch is required";
    return;
  }
  status_.state = GuardState::Degraded;
  status_.reason = reason;
}

void ClockMapper::authorityFailure(const std::string& reason, bool immediate_loss) {
  recordFailure(reason, immediate_loss);
}

TimestampEnvelope ClockMapper::makeEnvelope(const Observation& observation,
                                            bool accepted,
                                            const std::string& reason) const {
  TimestampEnvelope envelope;
  envelope.slot = observation.slot;
  envelope.source_body = route_.source_body;
  envelope.canonical_body = route_.canonical_body;
  envelope.source_domain = observation.source_domain;
  envelope.stream = observation.stream;
  envelope.epoch = status_.epoch;
  envelope.raw_source_stamp_ns = observation.raw_source_stamp_ns;
  envelope.mapped_session_stamp_ns = accepted ? status_.last_session_stamp_ns : 0U;
  envelope.receive_monotonic_stamp_ns = observation.receive_monotonic_stamp_ns;
  envelope.authority_age_ns = observation.authority_age_ns;
  envelope.gazebo_clock_skew_ns = observation.gazebo_clock_skew_ns;
  envelope.offset_ns = status_.offset_ns;
  envelope.drift_ppm = status_.drift_ppm;
  envelope.jitter_ns = status_.jitter_ns;
  envelope.uncertainty_ns = status_.uncertainty_ns;
  envelope.state = status_.state;
  envelope.accepted = accepted;
  envelope.canonical_publish_allowed = accepted && status_.state == GuardState::Locked;
  envelope.reason = reason;
  return envelope;
}

TimestampEnvelope ClockMapper::rejectedEnvelope(
    const Observation& observation,
    const std::string& reason) const {
  return makeEnvelope(observation, false, reason);
}

TimestampEnvelope ClockMapper::reject(const Observation& observation,
                                      const std::string& reason,
                                      bool immediate_loss) {
  recordFailure(reason, immediate_loss);
  return makeEnvelope(observation, false, status_.reason);
}

TimestampEnvelope ClockMapper::observe(const Observation& observation) {
  if (observation.slot != route_.slot ||
      observation.source_body != route_.source_body ||
      observation.source_domain != route_.source_domain) {
    return reject(observation, "observation does not match its frozen route", true);
  }
  if (status_.epoch == 0U) {
    return reject(observation, "epoch has not been started", false);
  }
  if (status_.state == GuardState::Lost) {
    return makeEnvelope(observation, false,
                        "clock guard is lost; a new Core-started process with a strictly newer epoch is required");
  }
  if (observation.fallback_source) {
    return reject(observation, "fallback clock sources are forbidden", true);
  }
  const std::size_t index = streamIndex(observation.stream);
  const auto& timeline = timelines_[index];
  if (observation.receive_monotonic_stamp_ns == 0U ||
      observation.receive_monotonic_stamp_ns < epoch_start_monotonic_stamp_ns_) {
    return reject(observation, "invalid steady-clock receive stamp", true);
  }
  for (const auto& required_timeline : timelines_) {
    if (!required_timeline.seen &&
        observation.receive_monotonic_stamp_ns -
                epoch_start_monotonic_stamp_ns_ >
            policy_.startup_lock_timeout_ns) {
      return reject(
          observation,
          "required VRPN stream did not start before the frozen startup lock timeout",
          true);
    }
  }
  if (observation.receive_session_stamp_ns == 0U) {
    return reject(observation, "zero Session receive stamp is invalid", true);
  }
  if (observation.raw_source_stamp_ns == 0U) {
    if (route_.source_domain == SourceDomain::Simulation && !timeline.seen &&
        status_.state == GuardState::Initializing) {
      status_.reason = "waiting for the first positive Gazebo sample timestamp";
      return makeEnvelope(observation, false, status_.reason);
    }
    return reject(observation, "zero source timestamp is invalid after stream start", true);
  }
  if (timeline.seen && observation.raw_source_stamp_ns <= timeline.raw_ns) {
    return reject(observation, "source timestamp repeated or moved backwards", true);
  }
  if (timeline.seen &&
      observation.receive_monotonic_stamp_ns <= timeline.monotonic_ns) {
    return reject(observation, "steady receive timestamp repeated or moved backwards", true);
  }

  const bool prior_affine_estimator_ready =
      mapping_ == ClockMapping::AffineToSession &&
      samples_.size() >= policy_.min_lock_samples;
  if (prior_affine_estimator_ready && have_estimate_) {
    const Estimate prediction =
        estimateAffine(samples_, observation.raw_source_stamp_ns);
    const std::int64_t prediction_error =
        observation.receive_session_stamp_ns >= prediction.mapped_session_ns
            ? static_cast<std::int64_t>(std::min<std::uint64_t>(
                  observation.receive_session_stamp_ns - prediction.mapped_session_ns,
                  static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())))
            : -static_cast<std::int64_t>(std::min<std::uint64_t>(
                  prediction.mapped_session_ns - observation.receive_session_stamp_ns,
                  static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())));
    if (absoluteUnsigned(prediction_error) > policy_.max_sample_age_ns) {
      return reject(observation,
                    "sample is stale or implausibly ahead of Session time", false);
    }
  }

  Estimate next;
  std::deque<Sample> candidate = samples_;
  if (mapping_ == ClockMapping::Identity) {
    next = estimateIdentity(observation.raw_source_stamp_ns);
  } else {
    const bool raw_stamp_already_fitted =
        std::any_of(candidate.begin(), candidate.end(), [&](const Sample& sample) {
          return sample.raw_ns == observation.raw_source_stamp_ns;
        });
    if (!raw_stamp_already_fitted) {
      candidate.push_back(
          {observation.raw_source_stamp_ns, observation.receive_session_stamp_ns});
    }
    while (candidate.size() > *policy_.estimator_window) {
      candidate.pop_front();
    }
    next = estimateAffine(candidate, observation.raw_source_stamp_ns);
  }

  const bool estimator_quality_ready =
      mapping_ != ClockMapping::AffineToSession ||
      candidate.size() >= policy_.min_lock_samples;
  std::vector<std::string> violations;
  if (mapping_ == ClockMapping::AffineToSession &&
      estimator_quality_ready) {
    if (have_estimate_ &&
        absoluteDifference(next.offset_ns, status_.offset_ns) >
            *policy_.max_offset_step_ns) {
      violations.emplace_back("offset step exceeds frozen threshold");
    }
    if (!std::isfinite(next.drift_ppm) ||
        std::fabs(next.drift_ppm) > *policy_.max_drift_ppm) {
      violations.emplace_back("drift exceeds frozen threshold");
    }
    if (next.jitter_ns > *policy_.max_jitter_ns) {
      violations.emplace_back("jitter exceeds frozen threshold");
    }
  }
  if (estimator_quality_ready &&
      next.uncertainty_ns > policy_.max_uncertainty_ns) {
    violations.emplace_back("uncertainty exceeds frozen threshold");
  }
  if (next.mapped_session_ns == 0U) {
    violations.emplace_back("mapped Session stamp is out of range");
  }
  if (!violations.empty()) {
    std::ostringstream reason;
    for (std::size_t i = 0; i < violations.size(); ++i) {
      if (i != 0U) {
        reason << "; ";
      }
      reason << violations[i];
    }
    return reject(observation, reason.str(), false);
  }

  if (mapping_ == ClockMapping::AffineToSession) {
    samples_ = std::move(candidate);
  }
  have_estimate_ = true;
  auto& mutable_timeline = timelines_[index];
  mutable_timeline.seen = true;
  mutable_timeline.raw_ns = observation.raw_source_stamp_ns;
  mutable_timeline.session_ns = observation.receive_session_stamp_ns;
  mutable_timeline.monotonic_ns = observation.receive_monotonic_stamp_ns;
  status_.offset_ns = next.offset_ns;
  status_.drift_ppm = next.drift_ppm;
  status_.jitter_ns = next.jitter_ns;
  status_.uncertainty_ns = next.uncertainty_ns;
  status_.last_raw_source_stamp_ns = observation.raw_source_stamp_ns;
  status_.last_session_stamp_ns = next.mapped_session_ns;
  status_.last_receive_monotonic_stamp_ns = observation.receive_monotonic_stamp_ns;
  status_.authority_age_ns = observation.authority_age_ns;
  status_.gazebo_clock_skew_ns = observation.gazebo_clock_skew_ns;
  status_.consecutive_failures = 0U;

  if (observation.raw_source_stamp_ns > last_counted_healthy_raw_ns_) {
    last_counted_healthy_raw_ns_ = observation.raw_source_stamp_ns;
    if (status_.healthy_samples < std::numeric_limits<std::uint32_t>::max()) {
      ++status_.healthy_samples;
    }
    if (status_.state == GuardState::Degraded &&
        recovery_samples_ < std::numeric_limits<std::uint32_t>::max()) {
      ++recovery_samples_;
    }
  }

  const bool required_streams_seen = timelines_[0].seen && timelines_[1].seen;
  if (status_.state == GuardState::Initializing && required_streams_seen &&
      status_.healthy_samples >= policy_.min_lock_samples) {
    status_.state = GuardState::Locked;
    status_.reason = mapping_ == ClockMapping::Identity
                         ? "identity mapping locked to Gazebo Session time"
                         : "affine source-to-Session mapping locked";
  } else if (status_.state == GuardState::Degraded && required_streams_seen &&
             recovery_samples_ >= policy_.recover_lock_samples) {
    status_.state = GuardState::Locked;
    status_.reason = "clock mapping relocked within the current non-lost epoch";
    recovery_samples_ = 0U;
  } else if (status_.state == GuardState::Initializing) {
    if (!required_streams_seen) {
      status_.reason = "waiting for both frozen pose and twist streams";
    } else {
      status_.reason = mapping_ == ClockMapping::Identity
                           ? "collecting identity-mapped source samples"
                           : "collecting source-to-Session affine samples";
    }
  } else if (status_.state == GuardState::Degraded) {
    status_.reason =
        "clock mapping is healthy but has not met the frozen recovery count";
  } else {
    status_.reason = "clock mapping locked";
  }

  return makeEnvelope(observation, true, status_.reason);
}

}  // namespace xgc_session_clock_guard
