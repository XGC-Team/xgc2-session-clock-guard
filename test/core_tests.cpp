#include "xgc_session_clock_guard/frozen_config.hpp"
#include "xgc_session_clock_guard/healthcheck.hpp"
#include "xgc_session_clock_guard/epoch_fence.hpp"
#include "xgc_session_clock_guard/session_clock_guard.hpp"

#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace guard = xgc_session_clock_guard;

namespace {

int failures = 0;

void check(bool condition, const std::string& message) {
  if (!condition) {
    ++failures;
    std::cerr << "FAIL: " << message << '\n';
  }
}

template <typename Callable>
void expectConfigError(Callable callable, const std::string& message) {
  try {
    callable();
    check(false, message + " (did not throw)");
  } catch (const guard::ConfigError&) {
  }
}

void replaceOnce(std::string* value,
                 const std::string& from,
                 const std::string& to) {
  const auto position = value->find(from);
  if (position == std::string::npos) {
    throw std::runtime_error("test fixture replacement did not find: " + from);
  }
  value->replace(position, from.size(), to);
}

void replaceAll(std::string* value,
                const std::string& from,
                const std::string& to) {
  std::size_t position = 0U;
  std::size_t replacements = 0U;
  while ((position = value->find(from, position)) != std::string::npos) {
    value->replace(position, from.size(), to);
    position += to.size();
    ++replacements;
  }
  if (replacements == 0U) {
    throw std::runtime_error("test fixture replacement did not find: " + from);
  }
}

std::string simulationConfig() {
  return
      "schema=xgc.session-clock-guard.config.v2\n"
      "policy_revision=xgc.session-clock-guard.builtin-policy.v1\n"
      "session_id=session-simulation-001\n"
      "session_contract_sha256=aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\n"
      "run_mode=simulation\n"
      "session_clock.authority=gazebo-simulation\n"
      "session_clock.mapping=identity\n"
      "vrpn.wire_time_resolution_ns=1000\n"
      "delay.measurement_enabled=true\n"
      "delay.timestamp_policy=sample_time\n"
      "threshold.min_lock_samples=2\n"
      "threshold.recover_lock_samples=2\n"
      "threshold.lost_after_failures=2\n"
      "threshold.startup_lock_timeout_ns=500000000\n"
      "threshold.max_uncertainty_ns=10000000\n"
      "threshold.max_sample_age_ns=50000000\n"
      "threshold.max_authority_age_ns=50000000\n"
      "threshold.max_gazebo_clock_skew_ns=20000000\n"
      "threshold.guard_poll_period_ns=25000000\n"
      "io.queue_depth=16\n"
      "route.px4-01.source_domain=simulation\n"
      "route.px4-01.source_body=uav1\n"
      "route.px4-01.canonical_body=uav1\n"
      "route.px4-01.sample_period_ns=10000000\n"
      "route.px4-01.source_uncertainty_ns=6000000\n";
}

std::string physicalConfig() {
  return
      "schema=xgc.session-clock-guard.config.v2\n"
      "policy_revision=xgc.session-clock-guard.builtin-policy.v1\n"
      "session_id=session-physical-001\n"
      "session_contract_sha256=bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb\n"
      "run_mode=physical\n"
      "session_clock.authority=station-wall-monotonic\n"
      "session_clock.mapping=affine-to-session\n"
      "vrpn.wire_time_resolution_ns=1000\n"
      "delay.measurement_enabled=true\n"
      "delay.timestamp_policy=sample_time\n"
      "threshold.estimator_window=8\n"
      "threshold.min_lock_samples=2\n"
      "threshold.recover_lock_samples=2\n"
      "threshold.lost_after_failures=2\n"
      "threshold.startup_lock_timeout_ns=500000000\n"
      "threshold.max_offset_step_ns=5000000\n"
      "threshold.max_drift_ppm=5000\n"
      "threshold.max_jitter_ns=2000000\n"
      "threshold.max_uncertainty_ns=10000000\n"
      "threshold.max_sample_age_ns=50000000\n"
      "threshold.max_authority_age_ns=50000000\n"
      "threshold.max_station_wall_error_ns=20000000\n"
      "threshold.max_station_wall_step_ns=5000000\n"
      "threshold.guard_poll_period_ns=25000000\n"
      "io.queue_depth=16\n"
      "route.scout-01.source_domain=physical\n"
      "route.scout-01.source_body=ugv1\n"
      "route.scout-01.canonical_body=ugv1\n"
      "route.scout-01.sample_period_ns=10000000\n"
      "route.scout-01.source_uncertainty_ns=6000000\n";
}

std::string hybridConfig() {
  return
      "schema=xgc.session-clock-guard.config.v2\n"
      "policy_revision=xgc.session-clock-guard.builtin-policy.v1\n"
      "session_id=session-hybrid-001\n"
      "session_contract_sha256=cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc\n"
      "run_mode=hybrid\n"
      "session_clock.authority=station-wall-monotonic\n"
      "session_clock.mapping=affine-to-session\n"
      "vrpn.wire_time_resolution_ns=1000\n"
      "delay.measurement_enabled=true\n"
      "delay.timestamp_policy=sample_time\n"
      "threshold.estimator_window=8\n"
      "threshold.min_lock_samples=2\n"
      "threshold.recover_lock_samples=2\n"
      "threshold.lost_after_failures=2\n"
      "threshold.startup_lock_timeout_ns=500000000\n"
      "threshold.max_offset_step_ns=5000000\n"
      "threshold.max_drift_ppm=5000\n"
      "threshold.max_jitter_ns=2000000\n"
      "threshold.max_uncertainty_ns=10000000\n"
      "threshold.max_sample_age_ns=50000000\n"
      "threshold.max_authority_age_ns=50000000\n"
      "threshold.max_gazebo_clock_skew_ns=20000000\n"
      "threshold.max_station_wall_error_ns=20000000\n"
      "threshold.max_station_wall_step_ns=5000000\n"
      "threshold.min_gazebo_real_time_factor=0.8\n"
      "threshold.max_gazebo_real_time_factor=1.2\n"
      "threshold.guard_poll_period_ns=25000000\n"
      "io.queue_depth=16\n"
      "route.px4-01.source_domain=simulation\n"
      "route.px4-01.source_body=uav1\n"
      "route.px4-01.canonical_body=uav1\n"
      "route.px4-01.sample_period_ns=10000000\n"
      "route.px4-01.source_uncertainty_ns=6000000\n"
      "route.scout-01.source_domain=physical\n"
      "route.scout-01.source_body=ugv1\n"
      "route.scout-01.canonical_body=uav7\n"
      "route.scout-01.sample_period_ns=10000000\n"
      "route.scout-01.source_uncertainty_ns=6000000\n";
}

std::string withoutRoutes(std::string config) {
  const auto routes = config.find("route.");
  if (routes == std::string::npos) {
    throw std::runtime_error("test fixture has no routes to remove");
  }
  config.erase(routes);
  return config;
}

guard::FrozenConfig parse(const std::string& text) {
  return guard::FrozenConfigLoader::parse(text, std::string(64U, 'd'));
}

std::string epochState(const guard::FrozenConfig& config,
                       const std::string& epoch) {
  return "{\"epochId\":\"" + epoch +
         "\",\"jobId\":\"job-1\",\"policySha256\":\"" +
         config.policy_sha256 +
         "\",\"schema\":\"xgc.session-clock-policy.epoch-state.v1\","
         "\"sessionContractSha256\":\"" +
         config.session_contract_sha256 + "\",\"sessionId\":\"" +
         config.session_id + "\",\"targetId\":\"local\"}\n";
}

guard::Observation observation(const std::string& slot,
                               const std::string& source_body,
                               guard::SourceDomain domain,
                               std::uint64_t raw_ns,
                               std::uint64_t monotonic_ns,
                               std::uint64_t wall_ns,
                               guard::StreamKind stream = guard::StreamKind::Pose) {
  guard::Observation value;
  value.slot = slot;
  value.source_body = source_body;
  value.source_domain = domain;
  value.stream = stream;
  value.raw_source_stamp_ns = raw_ns;
  value.receive_monotonic_stamp_ns = monotonic_ns;
  value.receive_system_wall_stamp_ns = wall_ns;
  return value;
}

guard::GazeboClockObservation gazeboClock(std::uint64_t gazebo_ns,
                                          std::uint64_t monotonic_ns,
                                          std::uint64_t wall_ns) {
  guard::GazeboClockObservation value;
  value.gazebo_stamp_ns = gazebo_ns;
  value.receive_monotonic_stamp_ns = monotonic_ns;
  value.receive_system_wall_stamp_ns = wall_ns;
  return value;
}

guard::AggregateAdmissionEvidence lockedEvidence(
    const guard::FrozenConfig& config,
    std::uint64_t epoch) {
  guard::AggregateAdmissionEvidence evidence;
  evidence.session_id = config.session_id;
  evidence.session_contract_sha256 = config.session_contract_sha256;
  evidence.policy_sha256 = config.policy_sha256;
  evidence.run_mode = config.run_mode;
  evidence.session_time_authority =
      guard::toString(config.session_time_authority);
  evidence.clock_mapping = guard::toString(config.clock_mapping);
  evidence.status_sequence = 10U;
  evidence.epoch = epoch;
  evidence.state = guard::GuardState::Locked;
  evidence.authority_state = guard::GuardState::Locked;
  evidence.authority_healthy = true;
  evidence.sensitive_output_allowed = true;
  if (config.run_mode == "simulation" || config.run_mode == "hybrid") {
    evidence.gazebo_clock_required = true;
    evidence.gazebo_clock_ready = true;
    evidence.gazebo_clock_topic = guard::gazeboClockTopic(config);
    evidence.last_gazebo_clock_stamp_ns = 1000000000ULL;
  }
  if (config.run_mode == "hybrid") {
    evidence.gazebo_real_time_factor = 1.0;
  }
  evidence.required_routes =
      static_cast<std::uint32_t>(config.routes.size());
  evidence.locked_routes = static_cast<std::uint32_t>(config.routes.size());
  evidence.vrpn_wire_resolution_ns = config.vrpn_wire_resolution_ns;
  evidence.measurement_delay_enabled = config.measurement_delay_enabled;
  evidence.delay_timestamp_policy = config.delay_timestamp_policy;
  return evidence;
}

void testSha256() {
  check(guard::sha256Hex("") ==
            "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855",
        "SHA-256 empty-vector mismatch");
  check(guard::sha256Hex("abc") ==
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad",
        "SHA-256 abc-vector mismatch");
}

void testFrozenModesRoutesAndTopics() {
  const auto simulation = parse(simulationConfig());
  const auto physical = parse(physicalConfig());
  const auto hybrid = parse(hybridConfig());
  check(simulation.session_time_authority ==
            guard::SessionTimeAuthority::GazeboSimulation &&
            simulation.clock_mapping == guard::ClockMapping::Identity,
        "Simulation authority/mapping pair was not preserved");
  check(physical.session_time_authority ==
            guard::SessionTimeAuthority::StationWallMonotonic &&
            physical.clock_mapping == guard::ClockMapping::AffineToSession,
        "Physical authority/mapping pair was not preserved");
  check(hybrid.routes.size() == 2U, "Hybrid route set was not preserved");
  check(simulation.thresholds.startup_lock_timeout_ns == 500000000ULL &&
            physical.thresholds.startup_lock_timeout_ns == 500000000ULL &&
            hybrid.thresholds.startup_lock_timeout_ns == 500000000ULL,
        "common startup lock timeout was not preserved in all three modes");
  check(guard::gazeboClockTopic(simulation) == "/clock",
        "Simulation must consume the global Gazebo clock");
  check(guard::gazeboClockTopic(hybrid) == "/xgc/source/gazebo/clock",
        "Hybrid must consume the private Gazebo clock");
  check(guard::gazeboClockTopic(physical).empty(),
        "Physical must not consume a Gazebo clock");
  check(guard::rawPoseTopic(hybrid.routes[1]) ==
            "/xgc/source/vrpn/physical/ugv1/pose",
        "raw topic must use source_body");
  check(guard::canonicalTwistTopic(hybrid.routes[1]) ==
            "/vrpn_client_node/uav7/twist",
        "canonical topic must use canonical_body");
  check(guard::statusTopic(hybrid.routes[1]) ==
            "/xgc/session/clock/vrpn/uav7/status",
        "SlotID must never be interpolated into a ROS topic");

  auto wrong_authority = simulationConfig();
  replaceOnce(&wrong_authority, "session_clock.authority=gazebo-simulation",
              "session_clock.authority=station-wall-monotonic");
  expectConfigError([&]() { parse(wrong_authority); },
                    "Simulation with station authority must fail closed");

  auto simulation_with_wall_field = simulationConfig();
  replaceOnce(&simulation_with_wall_field,
              "threshold.guard_poll_period_ns=25000000",
              "threshold.max_station_wall_error_ns=20000000\n"
              "threshold.guard_poll_period_ns=25000000");
  expectConfigError([&]() { parse(simulation_with_wall_field); },
                    "mode-mismatched Simulation wall threshold must be rejected");

  auto physical_with_gazebo_field = physicalConfig();
  replaceOnce(&physical_with_gazebo_field,
              "threshold.max_station_wall_error_ns=20000000",
              "threshold.max_gazebo_clock_skew_ns=20000000\n"
              "threshold.max_station_wall_error_ns=20000000");
  expectConfigError([&]() { parse(physical_with_gazebo_field); },
                    "mode-mismatched Physical Gazebo threshold must be rejected");

  auto incomplete_hybrid = hybridConfig();
  replaceOnce(&incomplete_hybrid,
              "threshold.max_gazebo_real_time_factor=1.2\n", "");
  expectConfigError([&]() { parse(incomplete_hybrid); },
                    "Hybrid with one RTF bound missing must fail closed");

  auto duplicate_canonical = hybridConfig();
  replaceOnce(&duplicate_canonical, "route.scout-01.canonical_body=uav7",
              "route.scout-01.canonical_body=uav1");
  expectConfigError([&]() { parse(duplicate_canonical); },
                    "duplicate canonical bodies must fail closed");

  auto duplicate_raw = hybridConfig();
  replaceOnce(&duplicate_raw,
              "route.scout-01.source_uncertainty_ns=6000000",
              "route.scout-01.source_uncertainty_ns=6000000\n"
              "route.scout-02.source_domain=physical\n"
              "route.scout-02.source_body=ugv1\n"
              "route.scout-02.canonical_body=uav8\n"
              "route.scout-02.sample_period_ns=10000000\n"
              "route.scout-02.source_uncertainty_ns=6000000");
  expectConfigError([&]() { parse(duplicate_raw); },
                    "duplicate raw domain/body sources must fail closed");

  auto illegal_body = hybridConfig();
  replaceOnce(&illegal_body, "route.scout-01.source_body=ugv1",
              "route.scout-01.source_body=ugv/1");
  expectConfigError([&]() { parse(illegal_body); },
                    "multi-segment source bodies must fail closed");

  auto send_time = hybridConfig();
  replaceOnce(&send_time, "delay.timestamp_policy=sample_time",
              "delay.timestamp_policy=send_time");
  expectConfigError([&]() { parse(send_time); },
                    "v24 send_time policy must fail closed");

  auto low_uncertainty = hybridConfig();
  replaceOnce(&low_uncertainty,
              "route.px4-01.source_uncertainty_ns=6000000",
              "route.px4-01.source_uncertainty_ns=1000");
  expectConfigError([&]() { parse(low_uncertainty); },
                    "uncertainty below quantization plus half-sample must fail closed");

  auto embedded_epoch = simulationConfig();
  replaceOnce(&embedded_epoch, "io.queue_depth=16",
              "epoch_id=41\nio.queue_depth=16");
  expectConfigError([&]() { parse(embedded_epoch); },
                    "epoch must remain a Session lifecycle input, not frozen config data");

  auto author_identity_override = simulationConfig();
  replaceOnce(&author_identity_override, "io.queue_depth=16",
              "author.session_id=forged\nio.queue_depth=16");
  expectConfigError([&]() { parse(author_identity_override); },
                    "author identity override keys must be unknown and rejected");

  auto wrong_policy_revision = simulationConfig();
  replaceOnce(&wrong_policy_revision,
              "policy_revision=xgc.session-clock-guard.builtin-policy.v1",
              "policy_revision=xgc.session-clock-guard.builtin-policy.v0");
  expectConfigError([&]() { parse(wrong_policy_revision); },
                    "unknown built-in policy revision must fail closed");

  auto mode_scoped_startup = simulationConfig();
  replaceOnce(&mode_scoped_startup,
              "threshold.startup_lock_timeout_ns=500000000",
              "simulation.threshold.startup_lock_timeout_ns=500000000");
  expectConfigError([&]() { parse(mode_scoped_startup); },
                    "startup timeout must be one common field, not a mode-scoped override");
}

void testCanonicalParserAndEpochText() {
  const std::string canonical = simulationConfig();
  for (const auto& malformed : {
           std::string("# comment\n") + canonical,
           std::string("\n") + canonical,
           std::string("schema =xgc.session-clock-guard.config.v2\n") +
               canonical.substr(canonical.find('\n') + 1U),
           canonical.substr(canonical.find('\n') + 1U) +
               canonical.substr(0U, canonical.find('\n') + 1U),
       }) {
    expectConfigError([&]() { parse(malformed); },
                      "noncanonical comments, blanks, padding, or order must fail");
  }

  auto leading_zero = canonical;
  replaceOnce(&leading_zero, "io.queue_depth=16", "io.queue_depth=016");
  expectConfigError([&]() { parse(leading_zero); },
                    "noncanonical unsigned spelling must fail");

  auto missing_startup = canonical;
  replaceOnce(&missing_startup,
              "threshold.startup_lock_timeout_ns=500000000\n", "");
  expectConfigError([&]() { parse(missing_startup); },
                    "missing common startup lock timeout must fail");

  auto startup_below_fixed_minimum = canonical;
  replaceOnce(&startup_below_fixed_minimum,
              "threshold.startup_lock_timeout_ns=500000000",
              "threshold.startup_lock_timeout_ns=249999999");
  expectConfigError([&]() { parse(startup_below_fixed_minimum); },
                    "startup timeout below 250 ms must fail");

  auto startup_above_fixed_maximum = canonical;
  replaceOnce(&startup_above_fixed_maximum,
              "threshold.startup_lock_timeout_ns=500000000",
              "threshold.startup_lock_timeout_ns=3000000001");
  expectConfigError([&]() { parse(startup_above_fixed_maximum); },
                    "startup timeout above 3 s must fail");

  auto startup_below_freshness = canonical;
  replaceOnce(&startup_below_freshness,
              "threshold.startup_lock_timeout_ns=500000000",
              "threshold.startup_lock_timeout_ns=250000000");
  replaceOnce(&startup_below_freshness,
              "threshold.max_sample_age_ns=50000000",
              "threshold.max_sample_age_ns=300000000");
  expectConfigError([&]() { parse(startup_below_freshness); },
                    "startup timeout below a runtime freshness bound must fail");

  auto reordered_startup = canonical;
  replaceOnce(&reordered_startup,
              "threshold.startup_lock_timeout_ns=500000000\n", "");
  replaceOnce(&reordered_startup,
              "threshold.max_authority_age_ns=50000000\n",
              "threshold.max_authority_age_ns=50000000\n"
              "threshold.startup_lock_timeout_ns=500000000\n");
  expectConfigError([&]() { parse(reordered_startup); },
                    "startup timeout outside canonical Core field order must fail");

  auto slow_poll = canonical;
  replaceOnce(&slow_poll, "threshold.guard_poll_period_ns=25000000",
              "threshold.guard_poll_period_ns=250000001");
  replaceOnce(&slow_poll, "threshold.max_sample_age_ns=50000000",
              "threshold.max_sample_age_ns=300000000");
  replaceOnce(&slow_poll, "threshold.max_authority_age_ns=50000000",
              "threshold.max_authority_age_ns=300000000");
  expectConfigError([&]() { parse(slow_poll); },
                    "guard poll periods over the 250 ms readiness bound must fail");

  auto padded_float = physicalConfig();
  replaceOnce(&padded_float, "threshold.max_drift_ppm=5000",
              "threshold.max_drift_ppm=5000.0");
  expectConfigError([&]() { parse(padded_float); },
                    "noncanonical float spelling must fail");

  auto subnormal = physicalConfig();
  replaceOnce(&subnormal, "threshold.max_drift_ppm=5000",
              "threshold.max_drift_ppm=0." + std::string(323U, '0') + "5");
  check(parse(subnormal).thresholds.max_drift_ppm.has_value(),
        "Core-canonical positive subnormal float64 must be accepted");

  auto long_session = canonical;
  replaceOnce(&long_session, "session_id=session-simulation-001",
              "session_id=" + std::string(65U, 's'));
  expectConfigError([&]() { parse(long_session); },
                    "Session IDs over 64 bytes must fail");

  auto long_slot = canonical;
  replaceAll(&long_slot, "route.px4-01.",
             "route." + std::string(129U, 's') + ".");
  expectConfigError([&]() { parse(long_slot); },
                    "SlotIDs over 128 bytes must fail");

  auto long_body = canonical;
  replaceOnce(&long_body, "route.px4-01.source_body=uav1",
              "route.px4-01.source_body=" + std::string(129U, 'u'));
  expectConfigError([&]() { parse(long_body); },
                    "ROS body segments over 128 bytes must fail");

  std::string oversized((1U << 20U) + 1U, 'x');
  oversized.back() = '\n';
  expectConfigError([&]() { parse(oversized); },
                    "policies over 1 MiB must fail before parsing");

  check(guard::parseEpochId("1") == 1U &&
            guard::parseEpochId("18446744073709551615") ==
                std::numeric_limits<std::uint64_t>::max(),
        "canonical epoch parser must accept the uint64 domain");
  for (const auto& invalid : {"", "0", "01", "+1", "-1", "1 "}) {
    try {
      (void)guard::parseEpochId(invalid);
      check(false, std::string("noncanonical epoch was accepted: ") + invalid);
    } catch (const std::invalid_argument&) {
    }
  }

  check(guard::parseRosPrivateEpochId("1") == 1U &&
            guard::parseRosPrivateEpochId("\"18446744073709551615\"") ==
                std::numeric_limits<std::uint64_t>::max(),
        "ROS private epoch parser must accept raw launch strings and exactly one formal double-quote wrapper");
  for (const auto& invalid : {"\"\"", "\"0\"", "\"01\"", "\"1", "1\"",
                              "\"\"1\"\"", "'1'", "!!str 1", "\"1 \""}) {
    try {
      (void)guard::parseRosPrivateEpochId(invalid);
      check(false,
            std::string("noncanonical ROS private epoch was accepted: ") +
                invalid);
    } catch (const std::invalid_argument&) {
    }
  }
}

void testEpochFenceCanonicalFileAndLoss() {
  const auto config = parse(withoutRoutes(physicalConfig()));
  const std::string state_bytes = epochState(config, "7");
  const auto parsed = guard::parseCanonicalEpochState(state_bytes);
  check(parsed.epoch == 7U && parsed.session_id == config.session_id,
        "canonical epoch state did not preserve trusted lineage");
  guard::validateEpochFenceState(parsed, config, 7U);
  try {
    guard::validateEpochFenceState(parsed, config, 8U);
    check(false, "a newer persisted epoch did not invalidate the old process");
  } catch (const guard::EpochFenceError&) {
  }

  auto unicode_trim_space = state_bytes;
  replaceOnce(&unicode_trim_space, "\"jobId\":\"job-1\"",
              "\"jobId\":\"job-1\\u00a0\"");
  try {
    (void)guard::parseCanonicalEpochState(unicode_trim_space);
    check(false, "Core-invalid trailing Unicode whitespace was accepted");
  } catch (const guard::EpochFenceError&) {
  }
  auto vertical_tab = state_bytes;
  replaceOnce(&vertical_tab, "\"targetId\":\"local\"",
              "\"targetId\":\"\\u000blocal\"");
  try {
    (void)guard::parseCanonicalEpochState(vertical_tab);
    check(false, "Core-invalid leading vertical tab was accepted");
  } catch (const guard::EpochFenceError&) {
  }

  for (const auto& malformed : {
           state_bytes.substr(0U, state_bytes.size() - 2U) +
               ",\"unknown\":\"x\"}\n",
           state_bytes.substr(0U, state_bytes.size() - 2U) +
               ",\"epochId\":\"7\"}\n",
           std::string("{\"jobId\":\"job-1\",\"epochId\":\"7\"}\n"),
           std::string("{ \"epochId\":\"7\"}\n"),
       }) {
    try {
      (void)guard::parseCanonicalEpochState(malformed);
      check(false, "noncanonical epoch-state.json was accepted");
    } catch (const guard::EpochFenceError&) {
    }
  }

  std::array<char, 64U> directory_template{};
  const std::string prefix = "/tmp/xgc-clock-fence-test.XXXXXX";
  std::copy(prefix.begin(), prefix.end(), directory_template.begin());
  char* directory = ::mkdtemp(directory_template.data());
  check(directory != nullptr, "could not create epoch fence test directory");
  if (directory == nullptr) {
    return;
  }
  const std::string root(directory);
  const std::string policy_file = root + "/policy.cfg";
  const std::string state_file = root + "/epoch-state.json";
  const std::string target_file = root + "/target.json";
  const std::string lock_file = root + "/epoch-state.lock";
  const std::string lock_target_file = root + "/lock-target";
  auto writeState = [&](const std::string& path, mode_t mode) {
    const int descriptor =
        ::open(path.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, mode);
    check(descriptor >= 0, "could not create epoch fence test file");
    if (descriptor >= 0) {
      const ssize_t written =
          ::write(descriptor, state_bytes.data(), state_bytes.size());
      check(written == static_cast<ssize_t>(state_bytes.size()),
            "could not write complete epoch fence fixture");
      ::close(descriptor);
      ::chmod(path.c_str(), mode);
    }
  };
  writeState(state_file, 0600);
  const int initial_lock =
      ::open(lock_file.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC,
             0600);
  check(initial_lock >= 0, "could not create epoch fence lock fixture");
  if (initial_lock >= 0) {
    ::close(initial_lock);
    ::chmod(lock_file.c_str(), 0600);
  }
  check(guard::epochLockPathForPolicy(policy_file) == lock_file,
        "epoch lock must be the fixed sibling of policyFile");
  const int core_allocator =
      ::open(lock_file.c_str(), O_RDWR | O_CLOEXEC | O_NOFOLLOW);
  check(core_allocator >= 0, "could not open Core allocation lock fixture");
  if (core_allocator >= 0) {
    check(::flock(core_allocator, LOCK_EX | LOCK_NB) == 0,
          "could not establish Core-style exclusive allocation fixture");
    try {
      guard::EpochFenceLease blocked_guard(policy_file);
      check(false, "Guard acquired a shared lease during Core allocation");
    } catch (const guard::EpochFenceError&) {
    }
    (void)::flock(core_allocator, LOCK_UN);
    ::close(core_allocator);
  }
  {
    guard::EpochFenceLease lease(policy_file);
    lease.validateCurrent();
    const int contender =
        ::open(lock_file.c_str(), O_RDWR | O_CLOEXEC | O_NOFOLLOW);
    check(contender >= 0, "could not open competing epoch lock fixture");
    if (contender >= 0) {
      check(::flock(contender, LOCK_EX | LOCK_NB) != 0 &&
                (errno == EWOULDBLOCK || errno == EAGAIN),
            "running Guard shared lease did not reject Core-style exclusive allocation");
      ::close(contender);
    }
  }
  const int after_guard =
      ::open(lock_file.c_str(), O_RDWR | O_CLOEXEC | O_NOFOLLOW);
  check(after_guard >= 0, "could not reopen released epoch lock fixture");
  if (after_guard >= 0) {
    check(::flock(after_guard, LOCK_EX | LOCK_NB) == 0,
          "exclusive epoch allocation must succeed after Guard lease release");
    (void)::flock(after_guard, LOCK_UN);
    ::close(after_guard);
  }
  check(guard::loadAndValidateEpochFence(policy_file, config, 7U).epoch == 7U,
        "secure 0600 epoch fence should validate");

  ::chmod(lock_file.c_str(), 0644);
  try {
    guard::EpochFenceLease bad_mode(policy_file);
    check(false, "non-0600 epoch fence lock was accepted");
  } catch (const guard::EpochFenceError&) {
  }
  ::chmod(lock_file.c_str(), 0600);

  const int nonempty_lock =
      ::open(lock_file.c_str(), O_WRONLY | O_CLOEXEC | O_NOFOLLOW);
  check(nonempty_lock >= 0, "could not open nonempty epoch lock fixture");
  if (nonempty_lock >= 0) {
    const char marker = 'x';
    check(::write(nonempty_lock, &marker, 1U) == 1,
          "could not write nonempty epoch lock fixture");
    ::close(nonempty_lock);
  }
  try {
    guard::EpochFenceLease nonempty(policy_file);
    check(false, "nonempty epoch fence lock was accepted");
  } catch (const guard::EpochFenceError&) {
  }
  const int truncate_lock =
      ::open(lock_file.c_str(), O_WRONLY | O_TRUNC | O_CLOEXEC | O_NOFOLLOW);
  check(truncate_lock >= 0, "could not restore empty epoch lock fixture");
  if (truncate_lock >= 0) {
    ::close(truncate_lock);
  }

  {
    guard::EpochFenceLease held_inode(policy_file);
    ::unlink(lock_file.c_str());
    const int replacement =
        ::open(lock_file.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC,
               0600);
    check(replacement >= 0, "could not replace epoch lock inode fixture");
    if (replacement >= 0) {
      ::close(replacement);
      ::chmod(lock_file.c_str(), 0600);
    }
    try {
      held_inode.validateCurrent();
      check(false, "replaced epoch lock inode still validated as the held lease");
    } catch (const guard::EpochFenceError&) {
    }
  }

  ::unlink(lock_file.c_str());
  const int lock_target =
      ::open(lock_target_file.c_str(),
             O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0600);
  check(lock_target >= 0, "could not create epoch lock symlink target");
  if (lock_target >= 0) {
    ::close(lock_target);
    ::chmod(lock_target_file.c_str(), 0600);
  }
  check(::symlink(lock_target_file.c_str(), lock_file.c_str()) == 0,
        "could not create epoch lock symlink fixture");
  try {
    guard::EpochFenceLease symlink_lock(policy_file);
    check(false, "symlink epoch fence lock was accepted");
  } catch (const guard::EpochFenceError&) {
  }
  ::unlink(lock_file.c_str());
  ::unlink(lock_target_file.c_str());
  check(::mkdir(lock_file.c_str(), 0700) == 0,
        "could not create nonregular epoch lock fixture");
  try {
    guard::EpochFenceLease directory_lock(policy_file);
    check(false, "directory epoch fence lock was accepted");
  } catch (const guard::EpochFenceError&) {
  }
  ::rmdir(lock_file.c_str());

  ::chmod(state_file.c_str(), 0644);
  try {
    (void)guard::loadAndValidateEpochFence(policy_file, config, 7U);
    check(false, "non-0600 epoch fence was accepted");
  } catch (const guard::EpochFenceError&) {
  }
  ::unlink(state_file.c_str());
  writeState(target_file, 0600);
  check(::symlink(target_file.c_str(), state_file.c_str()) == 0,
        "could not create epoch fence symlink fixture");
  try {
    (void)guard::loadAndValidateEpochFence(policy_file, config, 7U);
    check(false, "symlink epoch fence was accepted");
  } catch (const guard::EpochFenceError&) {
  }

  ::unlink(state_file.c_str());
  check(::mkdir(state_file.c_str(), 0700) == 0,
        "could not create nonregular epoch fence fixture");
  try {
    (void)guard::loadAndValidateEpochFence(policy_file, config, 7U);
    check(false, "directory epoch fence was accepted as a regular file");
  } catch (const guard::EpochFenceError&) {
  }
  ::rmdir(state_file.c_str());
  ::unlink(target_file.c_str());
  ::rmdir(root.c_str());

  guard::SessionClockGuard clock_guard(config, 7U, 1000000000ULL,
                                        10000000000ULL);
  clock_guard.failEpochFence("persisted epoch advanced", 1010000000ULL);
  check(clock_guard.aggregateState() == guard::GuardState::Lost &&
            !clock_guard.sensitiveOutputAllowed(),
        "epoch fence mismatch must immediately close the running Guard");
}

void testLiveReadinessEvidence() {
  const auto config = parse(withoutRoutes(simulationConfig()));
  auto evidence = lockedEvidence(config, 70U);

  check(guard::validateAggregatePublisherSet(
            {"/xgc_session_clock_guard"}, "/xgc_session_clock_guard")
            .empty(),
        "exact sole aggregate publisher must be accepted");
  check(!guard::validateAggregatePublisherSet(
             {"/external", "/xgc_session_clock_guard"},
             "/xgc_session_clock_guard")
              .empty(),
        "an external aggregate publisher must fail readiness");

  guard::LiveAdmissionTracker single_latched(
      config, 70U, "/xgc_session_clock_guard");
  single_latched.observe("/xgc_session_clock_guard", evidence);
  check(single_latched.acceptedSamples() == 1U && !single_latched.ready(),
        "one latched aggregate must never satisfy live readiness");

  guard::LiveAdmissionTracker duplicate_until_deadline(
      config, 70U, "/xgc_session_clock_guard");
  duplicate_until_deadline.observe("/xgc_session_clock_guard", evidence);
  for (std::size_t index = 0U; index < 20U; ++index) {
    duplicate_until_deadline.observe("/xgc_session_clock_guard", evidence);
  }
  check(duplicate_until_deadline.rejection().empty() &&
            duplicate_until_deadline.acceptedSamples() == 1U &&
            !duplicate_until_deadline.ready(),
        "same-identity duplicate aggregates through the deadline must wait without satisfying readiness");

  guard::LiveAdmissionTracker duplicate_then_advance(
      config, 70U, "/xgc_session_clock_guard");
  duplicate_then_advance.observe("/xgc_session_clock_guard", evidence);
  duplicate_then_advance.observe("/xgc_session_clock_guard", evidence);
  ++evidence.status_sequence;
  duplicate_then_advance.observe("/xgc_session_clock_guard", evidence);
  check(duplicate_then_advance.rejection().empty() &&
            duplicate_then_advance.ready(),
        "a same-identity duplicate followed by a greater sequence must satisfy readiness");

  guard::LiveAdmissionTracker regressed_sequence(
      config, 70U, "/xgc_session_clock_guard");
  regressed_sequence.observe("/xgc_session_clock_guard", evidence);
  --evidence.status_sequence;
  regressed_sequence.observe("/xgc_session_clock_guard", evidence);
  check(!regressed_sequence.rejection().empty(),
        "a strictly lower aggregate sequence must fail readiness immediately");

  guard::LiveAdmissionTracker external(
      config, 70U, "/xgc_session_clock_guard");
  external.observe("/external", evidence);
  check(!external.rejection().empty(),
        "connection caller identity must reject an external publisher");

  guard::LiveAdmissionTracker unknown(
      config, 70U, "/xgc_session_clock_guard");
  unknown.observe("unknown", evidence);
  check(!unknown.rejection().empty(),
        "a MessageEvent without a TCPROS callerid must fail readiness");

  guard::LiveAdmissionTracker live(config, 70U,
                                    "/xgc_session_clock_guard");
  live.observe("/xgc_session_clock_guard", evidence);
  ++evidence.status_sequence;
  live.observe("/xgc_session_clock_guard", evidence);
  check(live.ready(),
        "two locked /xgc_session_clock_guard aggregates with advancing sequence must pass");

  auto stale = lockedEvidence(config, 70U);
  stale.authority_age_ns = config.thresholds.max_authority_age_ns + 1U;
  check(!guard::validateLockedAdmission(stale, config, 70U).empty(),
        "readiness must enforce frozen authority diagnostic thresholds");
}

void testSimulationIdentityAuthorityAndEvents() {
  guard::SessionClockGuard clock_guard(parse(simulationConfig()), 10U,
                                        1000000000ULL, 5000000000ULL);
  std::string reason;
  auto events = clock_guard.takeEvents();
  check(events.size() == 1U &&
            events.front().kind == guard::GuardEventKind::NewEpoch,
        "new epoch event must be emitted");

  const auto before_clock = clock_guard.observe(observation(
      "px4-01", "uav1", guard::SourceDomain::Simulation, 1010000000ULL,
      1005000000ULL, 5005000000ULL));
  check(!before_clock.accepted &&
            before_clock.state == guard::GuardState::Initializing,
        "source data must remain closed before admitted Gazebo authority");

  check(!clock_guard.observeGazeboClock(
            gazeboClock(0U, 1006000000ULL, 5006000000ULL), &reason),
        "zero Gazebo startup time must wait without opening output");
  check(clock_guard.observeGazeboClock(
            gazeboClock(1010000000ULL, 1010000000ULL, 5010000000ULL), &reason),
        "first positive Simulation /clock must be accepted");
  const auto first = clock_guard.observe(observation(
      "px4-01", "uav1", guard::SourceDomain::Simulation, 1010000000ULL,
      1011000000ULL, 5011000000ULL));
  check(first.accepted && first.mapped_session_stamp_ns == 1010000000ULL,
        "Simulation mapping must preserve raw Gazebo sample time exactly");
  check(first.offset_ns == 0 && first.drift_ppm == 0.0 &&
            first.jitter_ns == 0U && first.uncertainty_ns == 6000000ULL,
        "identity mapping must report zero affine error and retain uncertainty floor");
  const auto first_twist = clock_guard.observe(observation(
      "px4-01", "uav1", guard::SourceDomain::Simulation, 1010000000ULL,
      1012000000ULL, 5012000000ULL, guard::StreamKind::Twist));
  check(first_twist.accepted && !first_twist.canonical_publish_allowed,
        "both frozen streams must be present before route lock");

  check(clock_guard.observeGazeboClock(
            gazeboClock(1020000000ULL, 1020000000ULL, 5020000000ULL), &reason),
        "monotonic Simulation /clock must continue");
  const auto second = clock_guard.observe(observation(
      "px4-01", "uav1", guard::SourceDomain::Simulation, 1020000000ULL,
      1021000000ULL, 5021000000ULL));
  check(second.canonical_publish_allowed &&
            clock_guard.aggregateState() == guard::GuardState::Locked,
        "Simulation output must open only after identity route lock");
  events = clock_guard.takeEvents();
  check(events.size() == 1U && events.front().kind == guard::GuardEventKind::Locked,
        "aggregate lock event must be emitted");

  const auto twist = clock_guard.observe(observation(
      "px4-01", "uav1", guard::SourceDomain::Simulation, 1020000000ULL,
      1022000000ULL, 5022000000ULL, guard::StreamKind::Twist));
  check(twist.accepted && twist.mapped_session_stamp_ns == 1020000000ULL,
        "pose and twist may share one source stamp on independent timelines");

  check(!clock_guard.observeGazeboClock(
            gazeboClock(1015000000ULL, 1030000000ULL, 5030000000ULL), &reason),
        "Gazebo rollback must fail closed");
  check(clock_guard.aggregateState() == guard::GuardState::Lost,
        "Gazebo rollback must immediately lose the epoch");
  events = clock_guard.takeEvents();
  check(events.size() == 1U && events.front().kind == guard::GuardEventKind::Lost,
        "lost transition event must be emitted");
  check(clock_guard.aggregateState() == guard::GuardState::Lost,
        "lost Guard must remain closed because runtime epoch advance is absent");
  guard::SessionClockGuard restarted(parse(simulationConfig()), 11U,
                                      1040000000ULL, 5040000000ULL);
  check(restarted.epoch() == 11U &&
            restarted.aggregateState() == guard::GuardState::Initializing,
        "a new Core-started process may begin the next persisted epoch");
}

void testAuthorityAgeAndSkewFailClosed() {
  guard::SessionClockGuard stale_guard(parse(simulationConfig()), 20U,
                                        1000000000ULL, 5000000000ULL);
  std::string reason;
  check(stale_guard.observeGazeboClock(
            gazeboClock(1010000000ULL, 1010000000ULL, 5010000000ULL), &reason),
        "stale test authority should start");
  stale_guard.poll(1070000000ULL, 5070000000ULL);
  check(stale_guard.aggregateState() == guard::GuardState::Degraded,
        "first steady-aged authority violation must degrade and close output");
  const auto degraded_events = stale_guard.takeEvents();
  check(degraded_events.size() == 2U &&
            degraded_events.back().kind == guard::GuardEventKind::Degraded,
        "authority-age degradation must emit an explicit transition event");
  stale_guard.poll(1095000000ULL, 5095000000ULL);
  check(stale_guard.aggregateState() == guard::GuardState::Lost,
        "repeated authority age violations must lose the epoch");

  guard::SessionClockGuard skew_guard(parse(simulationConfig()), 21U,
                                       1000000000ULL, 5000000000ULL);
  check(skew_guard.observeGazeboClock(
            gazeboClock(1010000000ULL, 1010000000ULL, 5010000000ULL), &reason),
        "skew test authority should start");
  const auto skew_one = skew_guard.observe(observation(
      "px4-01", "uav1", guard::SourceDomain::Simulation, 1050000000ULL,
      1011000000ULL, 5011000000ULL));
  check(!skew_one.accepted && skew_one.state == guard::GuardState::Degraded,
        "first Gazebo/source skew violation must close output");
  const auto skew_two = skew_guard.observe(observation(
      "px4-01", "uav1", guard::SourceDomain::Simulation, 1060000000ULL,
      1012000000ULL, 5012000000ULL));
  check(!skew_two.accepted && skew_two.state == guard::GuardState::Lost,
        "repeated Gazebo/source skew violations must lose the epoch");
}

void testStartupLockTimeoutIsSeparateFromRuntimeFreshness() {
  const auto zero_simulation = parse(withoutRoutes(simulationConfig()));
  guard::SessionClockGuard authority_within_grace(
      zero_simulation, 22U, 1000000000ULL, 5000000000ULL);
  authority_within_grace.poll(1300000000ULL, 5300000000ULL);
  check(authority_within_grace.aggregateState() ==
            guard::GuardState::Initializing,
        "missing initial Gazebo clock must remain initializing inside startup timeout");
  std::string reason;
  check(authority_within_grace.observeGazeboClock(
            gazeboClock(1400000000ULL, 1400000000ULL, 5400000000ULL),
            &reason) &&
            authority_within_grace.aggregateState() == guard::GuardState::Locked,
        "first positive Gazebo clock may arrive after runtime freshness but inside startup timeout");
  authority_within_grace.poll(1460000000ULL, 5460000000ULL);
  check(authority_within_grace.aggregateState() == guard::GuardState::Degraded,
        "seen Gazebo authority must immediately return to the 50 ms runtime freshness bound");

  guard::SessionClockGuard authority_timeout(
      zero_simulation, 23U, 1000000000ULL, 5000000000ULL);
  authority_timeout.poll(1500000001ULL, 5500000001ULL);
  check(authority_timeout.aggregateState() == guard::GuardState::Lost &&
            authority_timeout.authorityStatus().reason.find(
                "startup lock timeout") != std::string::npos,
        "missing initial Gazebo clock past startup timeout must hard-lose the epoch");
  check(!authority_timeout.observeGazeboClock(
            gazeboClock(1500000001ULL, 1500000001ULL, 5500000001ULL),
            &reason) &&
            authority_timeout.aggregateState() == guard::GuardState::Lost,
        "late Gazebo clock cannot recover a startup-timeout epoch");

  const auto physical = parse(physicalConfig());
  guard::SessionClockGuard route_within_grace(
      physical, 32U, 1000000000ULL, 10000000000ULL);
  route_within_grace.poll(1300000000ULL, 10300000000ULL);
  check(route_within_grace.aggregateState() == guard::GuardState::Initializing,
        "unseen pose/twist must remain initializing inside route startup timeout");
  const auto pose = route_within_grace.observe(observation(
      "scout-01", "ugv1", guard::SourceDomain::Physical, 2400000000ULL,
      1400000000ULL, 10400000000ULL));
  const auto twist = route_within_grace.observe(observation(
      "scout-01", "ugv1", guard::SourceDomain::Physical, 2400000000ULL,
      1410000000ULL, 10410000000ULL, guard::StreamKind::Twist));
  const auto second_pose = route_within_grace.observe(observation(
      "scout-01", "ugv1", guard::SourceDomain::Physical, 2420000000ULL,
      1420000000ULL, 10420000000ULL));
  check(pose.accepted && twist.accepted &&
            second_pose.canonical_publish_allowed,
        "both raw streams may first arrive after runtime freshness but inside startup timeout");
  route_within_grace.poll(1480000000ULL, 10480000000ULL);
  check(route_within_grace.aggregateState() == guard::GuardState::Degraded,
        "seen raw streams must immediately return to the 50 ms runtime freshness bound");

  guard::SessionClockGuard route_timeout(
      physical, 33U, 1000000000ULL, 10000000000ULL);
  route_timeout.poll(1500000001ULL, 10500000001ULL);
  check(route_timeout.aggregateState() == guard::GuardState::Lost &&
            route_timeout.routeStatus("scout-01").reason.find(
                "startup lock timeout") != std::string::npos,
        "missing pose/twist past startup timeout must hard-lose the epoch");
  const auto late_pose = route_timeout.observe(observation(
      "scout-01", "ugv1", guard::SourceDomain::Physical, 2500000000ULL,
      1500000001ULL, 10500000001ULL));
  check(!late_pose.accepted && late_pose.state == guard::GuardState::Lost,
        "late raw input cannot recover a startup-timeout epoch");
}

void testAffineQualityGatesStartAtFrozenMinimumSamples() {
  auto config_text = physicalConfig();
  replaceOnce(&config_text, "threshold.min_lock_samples=2",
              "threshold.min_lock_samples=8");
  replaceOnce(&config_text, "threshold.lost_after_failures=2",
              "threshold.lost_after_failures=3");

  const auto observe_pair = [](guard::SessionClockGuard* clock_guard,
                               std::uint64_t raw_ns,
                               std::uint64_t monotonic_ns) {
    const std::uint64_t wall_ns = 10000000000ULL +
                                  (monotonic_ns - 1000000000ULL);
    const auto pose = clock_guard->observe(observation(
        "scout-01", "ugv1", guard::SourceDomain::Physical, raw_ns,
        monotonic_ns, wall_ns));
    const auto twist = clock_guard->observe(observation(
        "scout-01", "ugv1", guard::SourceDomain::Physical, raw_ns,
        monotonic_ns + 1000ULL, wall_ns + 1000ULL,
        guard::StreamKind::Twist));
    return std::array<guard::TimestampEnvelope, 2>{pose, twist};
  };

  guard::SessionClockGuard jitter_guard(parse(config_text), 34U,
                                         1000000000ULL,
                                         10000000000ULL);
  const std::array<std::uint64_t, 8> startup_jitter_ns{
      0U, 200000U, 200000U, 200000U, 0U, 0U, 0U, 0U};
  for (std::size_t index = 0U; index < startup_jitter_ns.size(); ++index) {
    const std::uint64_t tick = static_cast<std::uint64_t>(index + 1U) *
                               10000000ULL;
    const auto envelopes = observe_pair(
        &jitter_guard, 2000000000ULL + tick,
        1000000000ULL + tick + startup_jitter_ns[index]);
    check(envelopes[0].accepted && envelopes[1].accepted,
          "sub-minimum 100 Hz affine startup jitter must collect without counting a quality failure");
    if (index == 1U) {
      check(envelopes[0].drift_ppm > 5000.0,
            "a provisional pre-minimum affine drift above the frozen limit must still be collected");
    }
    if (index + 1U < startup_jitter_ns.size()) {
      check(jitter_guard.aggregateState() == guard::GuardState::Initializing &&
                !envelopes[0].canonical_publish_allowed &&
                !envelopes[1].canonical_publish_allowed,
            "affine startup must remain initializing with canonical output closed before min_lock_samples");
    } else {
      check(envelopes[0].canonical_publish_allowed &&
                envelopes[1].canonical_publish_allowed,
            "the eighth healthy affine sample must publish canonical envelopes after passing the full gate");
    }
  }
  check(jitter_guard.aggregateState() == guard::GuardState::Locked &&
            jitter_guard.sensitiveOutputAllowed() &&
            jitter_guard.routeStatus("scout-01").consecutive_failures == 0U,
        "the first complete in-threshold affine estimate must lock at min_lock_samples");
  check(jitter_guard.routeStatus("scout-01").healthy_samples == 8U,
        "the eighth unique raw sample must be the first complete affine estimate");

  guard::SessionClockGuard over_limit_guard(parse(config_text), 35U,
                                             1000000000ULL,
                                             10000000000ULL);
  const std::array<std::uint64_t, 8> receive_offset_ns{
      0U, 10000000U, 10000000U, 10000000U,
      10000000U, 10000000U, 10000000U, 25000000U};
  for (std::size_t index = 0U; index < 7U; ++index) {
    const std::uint64_t tick = static_cast<std::uint64_t>(index + 1U) *
                               10000000ULL;
    const auto envelopes = observe_pair(
        &over_limit_guard, 3000000000ULL + tick,
        1000000000ULL + tick + receive_offset_ns[index]);
    check(envelopes[0].accepted && envelopes[1].accepted &&
              over_limit_guard.aggregateState() ==
                  guard::GuardState::Initializing &&
              !envelopes[0].canonical_publish_allowed &&
              !envelopes[1].canonical_publish_allowed,
          "an over-limit affine trend must still collect without canonical output before the frozen minimum");
    if (index >= 5U) {
      check(envelopes[0].uncertainty_ns > 10000000ULL,
            "pre-minimum affine uncertainty above the frozen limit must be deferred while canonical remains closed");
    }
  }
  const std::uint64_t full_tick = 80000000ULL;
  const std::uint64_t full_monotonic =
      1000000000ULL + full_tick + receive_offset_ns.back();
  const auto first_complete = over_limit_guard.observe(observation(
      "scout-01", "ugv1", guard::SourceDomain::Physical,
      3000000000ULL + full_tick, full_monotonic,
      10000000000ULL + (full_monotonic - 1000000000ULL)));
  check(!first_complete.accepted &&
            first_complete.state == guard::GuardState::Degraded &&
            !first_complete.canonical_publish_allowed &&
            first_complete.reason.find("offset step exceeds frozen threshold") !=
                std::string::npos &&
            first_complete.reason.find("drift exceeds frozen threshold") !=
                std::string::npos &&
            first_complete.reason.find("jitter exceeds frozen threshold") !=
                std::string::npos &&
            first_complete.reason.find("uncertainty exceeds frozen threshold") !=
                std::string::npos &&
            over_limit_guard.routeStatus("scout-01").consecutive_failures == 1U &&
            !over_limit_guard.sensitiveOutputAllowed(),
        "the first complete over-limit affine estimate must execute the full quality gate");

  guard::SessionClockGuard runtime_guard(parse(config_text), 36U,
                                          1000000000ULL,
                                          10000000000ULL);
  for (std::size_t index = 0U; index < 8U; ++index) {
    const std::uint64_t tick = static_cast<std::uint64_t>(index + 1U) *
                               10000000ULL;
    const auto envelopes = observe_pair(
        &runtime_guard, 4000000000ULL + tick,
        1000000000ULL + tick);
    check(envelopes[0].accepted && envelopes[1].accepted,
          "runtime-spike fixture must establish a healthy affine lock");
  }
  check(runtime_guard.aggregateState() == guard::GuardState::Locked,
        "runtime-spike fixture must be locked before injecting a receive-time spike");
  const std::uint64_t spike_monotonic = 1100000000ULL;
  const auto runtime_spike = runtime_guard.observe(observation(
      "scout-01", "ugv1", guard::SourceDomain::Physical,
      4090000000ULL, spike_monotonic,
      10000000000ULL + (spike_monotonic - 1000000000ULL)));
  check(!runtime_spike.accepted &&
            runtime_spike.state == guard::GuardState::Degraded &&
            !runtime_spike.canonical_publish_allowed &&
            runtime_spike.reason.find("frozen threshold") !=
                std::string::npos &&
            runtime_guard.routeStatus("scout-01").consecutive_failures == 1U &&
            !runtime_guard.sensitiveOutputAllowed(),
        "a runtime affine spike must be gated immediately after lock");
}

void testPhysicalStationAuthorityAndSteadyFreshness() {
  guard::SessionClockGuard clock_guard(parse(physicalConfig()), 30U,
                                        1000000000ULL, 10000000000ULL);
  const auto first = clock_guard.observe(observation(
      "scout-01", "ugv1", guard::SourceDomain::Physical, 2010000000ULL,
      1010000000ULL, 10010000000ULL));
  const auto first_twist = clock_guard.observe(observation(
      "scout-01", "ugv1", guard::SourceDomain::Physical, 2010000000ULL,
      1011000000ULL, 10011000000ULL, guard::StreamKind::Twist));
  const auto second = clock_guard.observe(observation(
      "scout-01", "ugv1", guard::SourceDomain::Physical, 2020000000ULL,
      1020000000ULL, 10020000000ULL));
  check(first.accepted && first_twist.accepted && second.canonical_publish_allowed,
        "healthy Physical affine samples must lock");
  check(second.mapped_session_stamp_ns == 10020000000ULL,
        "Physical mapping must target W0 plus steady elapsed");
  check(clock_guard.authorityStatus().station_wall_error_ns == 0,
        "matching system wall must have zero wall error");

  clock_guard.poll(1030000000ULL, 10080000000ULL);
  check(clock_guard.aggregateState() == guard::GuardState::Lost,
        "station wall error/step must immediately lose without re-anchoring");

  guard::SessionClockGuard silence_guard(parse(physicalConfig()), 31U,
                                          1000000000ULL, 10000000000ULL);
  for (const auto stream : {guard::StreamKind::Pose, guard::StreamKind::Twist}) {
    silence_guard.observe(observation(
        "scout-01", "ugv1", guard::SourceDomain::Physical, 2010000000ULL,
        1010000000ULL, 10010000000ULL, stream));
    silence_guard.observe(observation(
        "scout-01", "ugv1", guard::SourceDomain::Physical, 2020000000ULL,
        1020000000ULL, 10020000000ULL, stream));
  }
  check(silence_guard.sensitiveOutputAllowed(),
        "both Physical streams should lock before silence");
  silence_guard.poll(1080000000ULL, 10080000000ULL);
  check(silence_guard.aggregateState() == guard::GuardState::Degraded,
        "steady-clock stream age must degrade even with a perfectly tracking wall");
  silence_guard.poll(1105000000ULL, 10105000000ULL);
  check(silence_guard.aggregateState() == guard::GuardState::Lost,
        "repeated steady-clock silence must lose the epoch");
}

void testHybridPrivateClockRtfAndBodyProjection() {
  guard::SessionClockGuard clock_guard(parse(hybridConfig()), 40U,
                                        1000000000ULL, 10000000000ULL);
  std::string reason;
  check(clock_guard.observeGazeboClock(
            gazeboClock(2010000000ULL, 1010000000ULL, 10010000000ULL), &reason),
        "first private Hybrid clock sample should be admitted");
  check(!clock_guard.authorityStatus().healthy,
        "one Hybrid clock sample cannot establish RTF");
  check(clock_guard.observeGazeboClock(
            gazeboClock(2020000000ULL, 1020000000ULL, 10020000000ULL), &reason),
        "second real-time private clock sample should establish RTF");
  check(clock_guard.authorityStatus().healthy &&
            clock_guard.authorityStatus().gazebo_real_time_factor == 1.0,
        "Hybrid authority must report a healthy 1.0 RTF");

  auto sim_first = clock_guard.observe(observation(
      "px4-01", "uav1", guard::SourceDomain::Simulation, 2020000000ULL,
      1021000000ULL, 10021000000ULL));
  auto physical_first = clock_guard.observe(observation(
      "scout-01", "ugv1", guard::SourceDomain::Physical, 3020000000ULL,
      1022000000ULL, 10022000000ULL));
  check(sim_first.accepted && physical_first.accepted,
        "both Hybrid source domains must enter the same affine Session timeline");
  const auto sim_first_twist = clock_guard.observe(observation(
      "px4-01", "uav1", guard::SourceDomain::Simulation, 2020000000ULL,
      1023000000ULL, 10023000000ULL, guard::StreamKind::Twist));
  const auto physical_first_twist = clock_guard.observe(observation(
      "scout-01", "ugv1", guard::SourceDomain::Physical, 3020000000ULL,
      1024000000ULL, 10024000000ULL, guard::StreamKind::Twist));
  check(sim_first_twist.accepted && physical_first_twist.accepted,
        "Hybrid lock evidence must include pose and twist for each route");

  check(clock_guard.observeGazeboClock(
            gazeboClock(2030000000ULL, 1030000000ULL, 10030000000ULL), &reason),
        "next private Hybrid clock should remain within RTF bounds");
  const auto sim_second = clock_guard.observe(observation(
      "px4-01", "uav1", guard::SourceDomain::Simulation, 2030000000ULL,
      1031000000ULL, 10031000000ULL));
  const auto physical_second = clock_guard.observe(observation(
      "scout-01", "ugv1", guard::SourceDomain::Physical, 3030000000ULL,
      1032000000ULL, 10032000000ULL));
  check(sim_second.accepted && physical_second.canonical_publish_allowed,
        "Hybrid output must open only after both route mappings lock");
  check(physical_second.source_body == "ugv1" &&
            physical_second.canonical_body == "uav7",
        "Hybrid envelope must retain source lineage and canonical projection");

  check(!clock_guard.observeGazeboClock(
            gazeboClock(2130000000ULL, 1040000000ULL, 10040000000ULL), &reason),
        "out-of-bounds private Gazebo RTF must fail closed");
  check(clock_guard.aggregateState() == guard::GuardState::Degraded &&
            !clock_guard.sensitiveOutputAllowed(),
        "first Hybrid RTF violation must degrade aggregate output");
  check(!clock_guard.observeGazeboClock(
            gazeboClock(2140000000ULL, 1050000000ULL, 10050000000ULL), &reason),
        "repeated private Gazebo RTF violation must remain rejected");
  check(clock_guard.aggregateState() == guard::GuardState::Lost,
        "repeated Hybrid RTF violation must lose the epoch");
}

void testPureModeAuthorityOnlySessions() {
  const auto zero_simulation = parse(withoutRoutes(simulationConfig()));
  check(zero_simulation.routes.empty(),
        "pure Simulation must permit an authority-only zero-route contract");
  auto admission = lockedEvidence(zero_simulation, 60U);
  check(guard::validateLockedAdmission(admission, zero_simulation, 60U).empty(),
        "readiness must accept a locked digest-bound zero-route Simulation");
  admission.state = guard::GuardState::Initializing;
  check(!guard::validateLockedAdmission(admission, zero_simulation, 60U).empty(),
        "readiness must reject an initializing aggregate message");
  admission = lockedEvidence(zero_simulation, 60U);
  admission.policy_sha256 = std::string(64U, '0');
  check(!guard::validateLockedAdmission(admission, zero_simulation, 60U).empty(),
        "readiness must reject a stale or foreign policy digest");
  guard::SessionClockGuard simulation_guard(zero_simulation, 60U,
                                             1000000000ULL, 5000000000ULL);
  std::string reason;
  check(simulation_guard.aggregateState() == guard::GuardState::Initializing &&
            !simulation_guard.sensitiveOutputAllowed(),
        "authority-only Simulation must wait for admitted global /clock");
  check(simulation_guard.observeGazeboClock(
            gazeboClock(1010000000ULL, 1010000000ULL, 5010000000ULL),
            &reason),
        "authority-only Simulation must accept its first positive /clock");
  check(simulation_guard.aggregateState() == guard::GuardState::Locked &&
            simulation_guard.sensitiveOutputAllowed() &&
            simulation_guard.lockedRouteCount() == 0U,
        "authority-only Simulation must lock without inventing Robot routes");
  const auto simulation_events = simulation_guard.takeEvents();
  check(simulation_events.size() == 2U &&
            simulation_events.front().kind == guard::GuardEventKind::NewEpoch &&
            simulation_events.back().kind == guard::GuardEventKind::Locked,
        "authority-only Simulation must emit new-epoch then locked events");
  simulation_guard.poll(1070000000ULL, 5070000000ULL);
  check(simulation_guard.aggregateState() == guard::GuardState::Degraded,
        "authority-only Simulation must degrade on stale /clock");
  simulation_guard.poll(1095000000ULL, 5095000000ULL);
  check(simulation_guard.aggregateState() == guard::GuardState::Lost,
        "authority-only Simulation must lose after repeated stale /clock");

  const auto zero_physical = parse(withoutRoutes(physicalConfig()));
  check(zero_physical.routes.empty(),
        "pure Physical must permit an authority-only zero-route contract");
  guard::SessionClockGuard physical_guard(zero_physical, 61U,
                                           1000000000ULL, 10000000000ULL);
  check(physical_guard.aggregateState() == guard::GuardState::Locked &&
            physical_guard.sensitiveOutputAllowed() &&
            physical_guard.routeStatuses().empty(),
        "authority-only Physical must lock on its immutable station anchors");
  const auto physical_events = physical_guard.takeEvents();
  check(physical_events.size() == 2U &&
            physical_events.back().kind == guard::GuardEventKind::Locked,
        "authority-only Physical must emit an immediate locked event");
  physical_guard.poll(1010000000ULL, 10080000000ULL);
  check(physical_guard.aggregateState() == guard::GuardState::Lost,
        "authority-only Physical must lose immediately on station wall step");
  check(physical_guard.aggregateState() == guard::GuardState::Lost,
        "authority-only loss must remain closed without a runtime epoch entrypoint");

  expectConfigError([&]() { parse(withoutRoutes(hybridConfig())); },
                    "Hybrid must reject an authority-only zero-route contract");
}

}  // namespace

int main(int argc, char** argv) {
  testSha256();
  testFrozenModesRoutesAndTopics();
  testCanonicalParserAndEpochText();
  testEpochFenceCanonicalFileAndLoss();
  testLiveReadinessEvidence();
  testSimulationIdentityAuthorityAndEvents();
  testAuthorityAgeAndSkewFailClosed();
  testStartupLockTimeoutIsSeparateFromRuntimeFreshness();
  testAffineQualityGatesStartAtFrozenMinimumSamples();
  testPhysicalStationAuthorityAndSteadyFreshness();
  testHybridPrivateClockRtfAndBodyProjection();
  testPureModeAuthorityOnlySessions();
  if ((argc - 1) % 2 != 0) {
    check(false, "core test config arguments must be repeated <config> <sha256> pairs");
  } else {
    for (int index = 1; index + 1 < argc; index += 2) {
      try {
        const auto config =
            guard::FrozenConfigLoader::loadFile(argv[index], argv[index + 1]);
        check(!config.session_id.empty(),
              "verified example config did not preserve its Session identity");
      } catch (const std::exception& error) {
        check(false, std::string("verified example config failed to load: ") +
                         error.what());
      }
    }
  }
  if (failures != 0) {
    std::cerr << failures << " test assertion(s) failed\n";
    return EXIT_FAILURE;
  }
  std::cout << "session-clock-guard core tests passed\n";
  return EXIT_SUCCESS;
}
