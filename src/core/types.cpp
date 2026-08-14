#include "xgc_session_clock_guard/types.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>

namespace xgc_session_clock_guard {

namespace {

constexpr const char *kSimulationRawRoot = "/xgc/source/vrpn/simulation";
constexpr const char *kPhysicalRawRoot = "/xgc/source/vrpn/physical";
constexpr const char *kCanonicalRoot = "/vrpn_client_node";
constexpr const char *kSidecarRoot = "/xgc/session/clock/vrpn";
constexpr const char *kSimulationGazeboClockTopic = "/clock";
constexpr const char *kHybridGazeboClockTopic = "/xgc/source/gazebo/clock";

} // namespace

const char *toString(GuardState state) {
  switch (state) {
  case GuardState::Initializing:
    return "initializing";
  case GuardState::Locked:
    return "locked";
  case GuardState::Degraded:
    return "degraded";
  case GuardState::Lost:
    return "lost";
  }
  return "lost";
}

const char *toString(SourceDomain domain) {
  switch (domain) {
  case SourceDomain::Simulation:
    return "simulation";
  case SourceDomain::Physical:
    return "physical";
  }
  return "invalid";
}

const char *toString(StreamKind stream) {
  switch (stream) {
  case StreamKind::Pose:
    return "pose";
  case StreamKind::Twist:
    return "twist";
  }
  return "invalid";
}

const char *toString(SessionTimeAuthority authority) {
  switch (authority) {
  case SessionTimeAuthority::GazeboSimulation:
    return "gazebo-simulation";
  case SessionTimeAuthority::StationWallMonotonic:
    return "station-wall-monotonic";
  }
  return "invalid";
}

const char *toString(ClockMapping mapping) {
  switch (mapping) {
  case ClockMapping::Identity:
    return "identity";
  case ClockMapping::AffineToSession:
    return "affine-to-session";
  }
  return "invalid";
}

const char *toString(GuardEventKind kind) {
  switch (kind) {
  case GuardEventKind::NewEpoch:
    return "new_epoch";
  case GuardEventKind::Locked:
    return "locked";
  case GuardEventKind::Degraded:
    return "degraded";
  case GuardEventKind::Lost:
    return "lost";
  }
  return "lost";
}

SourceDomain parseSourceDomain(const std::string &text) {
  if (text == "simulation") {
    return SourceDomain::Simulation;
  }
  if (text == "physical") {
    return SourceDomain::Physical;
  }
  throw std::invalid_argument(
      "source_domain must be exactly simulation or physical");
}

SessionTimeAuthority parseSessionTimeAuthority(const std::string &text) {
  if (text == "gazebo-simulation") {
    return SessionTimeAuthority::GazeboSimulation;
  }
  if (text == "station-wall-monotonic") {
    return SessionTimeAuthority::StationWallMonotonic;
  }
  throw std::invalid_argument("session_clock.authority must be exactly "
                              "gazebo-simulation or station-wall-monotonic");
}

ClockMapping parseClockMapping(const std::string &text) {
  if (text == "identity") {
    return ClockMapping::Identity;
  }
  if (text == "affine-to-session") {
    return ClockMapping::AffineToSession;
  }
  throw std::invalid_argument(
      "session_clock.mapping must be exactly identity or affine-to-session");
}

std::uint64_t parseEpochId(const std::string &text) {
  if (text.empty() || text.front() < '1' || text.front() > '9' ||
      !std::all_of(text.begin(), text.end(), [](unsigned char character) {
        return character >= '0' && character <= '9';
      })) {
    throw std::invalid_argument(
        "epoch must be a nonzero canonical unsigned decimal string");
  }
  std::size_t consumed = 0U;
  try {
    const std::uint64_t epoch = std::stoull(text, &consumed, 10);
    if (consumed != text.size() || epoch == 0U) {
      throw std::invalid_argument(
          "epoch must be a nonzero canonical unsigned decimal string");
    }
    return epoch;
  } catch (const std::out_of_range &) {
    throw std::invalid_argument("epoch is outside the uint64 range");
  }
}

std::uint64_t parseRosPrivateEpochId(const std::string &text) {
  if (!text.empty() && (text.front() == '"' || text.back() == '"')) {
    if (text.size() < 3U || text.front() != '"' || text.back() != '"') {
      throw std::invalid_argument("ROS private epoch must use exactly one "
                                  "complete ASCII double-quote wrapper");
    }
    return parseEpochId(text.substr(1U, text.size() - 2U));
  }
  return parseEpochId(text);
}

std::string rawRoot(SourceDomain domain) {
  return domain == SourceDomain::Simulation ? kSimulationRawRoot
                                            : kPhysicalRawRoot;
}

std::string rawPoseTopic(const Route &route) {
  return rawRoot(route.source_domain) + "/" + route.source_body + "/pose";
}

std::string rawTwistTopic(const Route &route) {
  return rawRoot(route.source_domain) + "/" + route.source_body + "/twist";
}

std::string canonicalPoseTopic(const Route &route) {
  return std::string(kCanonicalRoot) + "/" + route.canonical_body + "/pose";
}

std::string canonicalTwistTopic(const Route &route) {
  return std::string(kCanonicalRoot) + "/" + route.canonical_body + "/twist";
}

std::string envelopeTopic(const Route &route) {
  return std::string(kSidecarRoot) + "/" + route.canonical_body + "/envelope";
}

std::string statusTopic(const Route &route) {
  return std::string(kSidecarRoot) + "/" + route.canonical_body + "/status";
}

std::string gazeboClockTopic(const FrozenConfig &config) {
  if (config.run_mode == "simulation") {
    return kSimulationGazeboClockTopic;
  }
  if (config.run_mode == "hybrid") {
    return kHybridGazeboClockTopic;
  }
  return {};
}

} // namespace xgc_session_clock_guard
