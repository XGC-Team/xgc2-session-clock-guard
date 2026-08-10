#pragma once

#include <array>
#include <cstdint>
#include <deque>
#include <string>

#include "xgc_session_clock_guard/types.hpp"

namespace xgc_session_clock_guard {

class SessionClockGuard;

class ClockMapper {
 public:
  ClockMapper(const Route& route,
              const ThresholdPolicy& policy,
              ClockMapping mapping);

  TimestampEnvelope observe(const Observation& observation);
  void poll(std::uint64_t monotonic_now_ns);
  void authorityFailure(const std::string& reason, bool immediate_loss);
  TimestampEnvelope rejectedEnvelope(const Observation& observation,
                                     const std::string& reason) const;
  const RouteStatus& status() const { return status_; }

 private:
  friend class SessionClockGuard;

  bool initializeEpoch(std::uint64_t epoch,
                       std::uint64_t session_start_stamp_ns,
                       std::uint64_t monotonic_start_stamp_ns,
                       std::string* reason);
  struct Sample {
    std::uint64_t raw_ns{0};
    std::uint64_t session_ns{0};
  };

  struct Estimate {
    std::int64_t offset_ns{0};
    double drift_ppm{0.0};
    std::uint64_t jitter_ns{0};
    std::uint64_t uncertainty_ns{0};
    std::uint64_t mapped_session_ns{0};
  };

  struct StreamTimeline {
    bool seen{false};
    std::uint64_t raw_ns{0};
    std::uint64_t session_ns{0};
    std::uint64_t monotonic_ns{0};
  };

  Estimate estimateAffine(const std::deque<Sample>& samples,
                          std::uint64_t raw_ns) const;
  Estimate estimateIdentity(std::uint64_t raw_ns) const;
  TimestampEnvelope reject(const Observation& observation,
                           const std::string& reason,
                           bool immediate_loss);
  void recordFailure(const std::string& reason, bool immediate_loss);
  TimestampEnvelope makeEnvelope(const Observation& observation,
                                 bool accepted,
                                 const std::string& reason) const;

  Route route_;
  ThresholdPolicy policy_;
  ClockMapping mapping_{ClockMapping::Identity};
  std::deque<Sample> samples_;
  std::array<StreamTimeline, 2> timelines_{};
  RouteStatus status_;
  bool have_estimate_{false};
  std::uint64_t epoch_start_session_stamp_ns_{0};
  std::uint64_t epoch_start_monotonic_stamp_ns_{0};
  std::uint64_t last_counted_healthy_raw_ns_{0};
  std::uint32_t recovery_samples_{0};
};

}  // namespace xgc_session_clock_guard
