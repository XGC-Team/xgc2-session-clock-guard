#include <gtest/gtest.h>

#include <xgc_session_clock_guard/ClockGuardAggregateStatus.h>
#include <xgc_session_clock_guard/ClockGuardStatus.h>
#include <xgc_session_clock_guard/ClockTimestampEnvelope.h>

#include "xgc_session_clock_guard/frozen_config.hpp"
#include "xgc_session_clock_guard/session_clock_guard.hpp"

namespace guard = xgc_session_clock_guard;

namespace {

std::string simulationConfig() {
  return "schema=xgc.session-clock-guard.config.v2\n"
         "policy_revision=xgc.session-clock-guard.builtin-policy.v2\n"
         "session_id=catkin-simulation\n"
         "session_contract_sha256="
         "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\n"
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

guard::Observation sample(std::uint64_t raw_ns, std::uint64_t monotonic_ns) {
  guard::Observation observation;
  observation.slot = "px4-01";
  observation.source_body = "uav1";
  observation.source_domain = guard::SourceDomain::Simulation;
  observation.stream = guard::StreamKind::Pose;
  observation.raw_source_stamp_ns = raw_ns;
  observation.receive_monotonic_stamp_ns = monotonic_ns;
  observation.receive_system_wall_stamp_ns = monotonic_ns + 4000000000ULL;
  return observation;
}

guard::GazeboClockObservation gazebo(std::uint64_t raw_ns,
                                     std::uint64_t monotonic_ns) {
  guard::GazeboClockObservation observation;
  observation.gazebo_stamp_ns = raw_ns;
  observation.receive_monotonic_stamp_ns = monotonic_ns;
  observation.receive_system_wall_stamp_ns = monotonic_ns + 4000000000ULL;
  return observation;
}

TEST(SessionClockGuardCatkin, Sha256AndStrictSimulationPolicy) {
  EXPECT_EQ("ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad",
            guard::sha256Hex("abc"));
  const auto config = guard::FrozenConfigLoader::parse(simulationConfig(),
                                                       std::string(64U, 'd'));
  EXPECT_EQ(guard::SessionTimeAuthority::GazeboSimulation,
            config.session_time_authority);
  EXPECT_EQ(guard::ClockMapping::Identity, config.clock_mapping);
  EXPECT_EQ("/xgc/session/clock/vrpn/uav1/status",
            guard::statusTopic(config.routes.front()));
}

TEST(SessionClockGuardCatkin, RosbagLineageMessagesCarryModeAndSequence) {
  xgc_session_clock_guard::ClockGuardStatus status;
  xgc_session_clock_guard::ClockTimestampEnvelope envelope;
  xgc_session_clock_guard::ClockGuardAggregateStatus aggregate;
  status.run_mode = "physical";
  envelope.run_mode = "hybrid";
  aggregate.status_sequence = 2U;
  EXPECT_EQ("physical", status.run_mode);
  EXPECT_EQ("hybrid", envelope.run_mode);
  EXPECT_EQ(2U, aggregate.status_sequence);
}

TEST(SessionClockGuardCatkin, IdentityRollbackNeedsStrictlyNewerEpoch) {
  auto config = guard::FrozenConfigLoader::parse(simulationConfig(),
                                                 std::string(64U, 'd'));
  guard::SessionClockGuard clock_guard(std::move(config), 40U, 1000000000ULL,
                                       5000000000ULL);
  std::string reason;
  ASSERT_TRUE(clock_guard.observeGazeboClock(
      gazebo(1010000000ULL, 1010000000ULL), &reason));
  const auto first = clock_guard.observe(sample(1010000000ULL, 1011000000ULL));
  EXPECT_TRUE(first.accepted);
  EXPECT_EQ(first.raw_source_stamp_ns, first.mapped_session_stamp_ns);
  EXPECT_FALSE(clock_guard.observeGazeboClock(
      gazebo(1000000000ULL, 1020000000ULL), &reason));
  EXPECT_EQ(guard::GuardState::Lost, clock_guard.aggregateState());
  guard::SessionClockGuard restarted(
      guard::FrozenConfigLoader::parse(simulationConfig(),
                                       std::string(64U, 'd')),
      41U, 1030000000ULL, 5030000000ULL);
  EXPECT_EQ(41U, restarted.epoch());
}

TEST(SessionClockGuardCatkin, ModeMismatchAndImpossibleUncertaintyFailClosed) {
  auto wrong_mapping = simulationConfig();
  const auto mapping = wrong_mapping.find("session_clock.mapping=identity");
  wrong_mapping.replace(mapping,
                        std::string("session_clock.mapping=identity").size(),
                        "session_clock.mapping=affine-to-session");
  EXPECT_THROW(
      guard::FrozenConfigLoader::parse(wrong_mapping, std::string(64U, 'd')),
      guard::ConfigError);

  auto impossible = simulationConfig();
  const auto uncertainty =
      impossible.find("route.px4-01.source_uncertainty_ns=6000000");
  impossible.replace(
      uncertainty,
      std::string("route.px4-01.source_uncertainty_ns=6000000").size(),
      "route.px4-01.source_uncertainty_ns=1000");
  EXPECT_THROW(
      guard::FrozenConfigLoader::parse(impossible, std::string(64U, 'd')),
      guard::ConfigError);
}

} // namespace

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
