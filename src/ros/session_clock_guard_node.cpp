#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>

#include <xgc_session_clock_guard/ClockGuardAggregateStatus.h>
#include <xgc_session_clock_guard/ClockGuardEvent.h>
#include <xgc_session_clock_guard/ClockGuardStatus.h>
#include <xgc_session_clock_guard/ClockTimestampEnvelope.h>

#include "xgc_session_clock_guard/epoch_fence.hpp"
#include "xgc_session_clock_guard/frozen_config.hpp"
#include "xgc_session_clock_guard/session_clock_guard.hpp"

namespace xgc_session_clock_guard {
namespace {

std::uint64_t systemWallNowNs() {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  const auto nanoseconds =
      std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
  if (nanoseconds <= 0) {
    throw std::runtime_error(
        "system wall clock returned a non-positive timestamp");
  }
  return static_cast<std::uint64_t>(nanoseconds);
}

std::uint64_t steadyNowNs() {
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  const auto nanoseconds =
      std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
  if (nanoseconds <= 0) {
    throw std::runtime_error("steady clock returned a non-positive timestamp");
  }
  return static_cast<std::uint64_t>(nanoseconds);
}

ros::Time rosTimeFromNs(std::uint64_t value) {
  ros::Time result;
  result.fromNSec(value);
  return result;
}

template <typename Value>
Value requiredParam(const ros::NodeHandle &private_node,
                    const std::string &name) {
  Value value;
  if (!private_node.getParam(name, value)) {
    throw std::runtime_error("required private parameter is missing: ~" + name);
  }
  return value;
}

} // namespace

class SessionClockGuardNode {
public:
  SessionClockGuardNode() : node_(), private_node_("~") {
    policy_file_ = requiredParam<std::string>(private_node_, "policy_file");
    const std::string policy_sha256 =
        requiredParam<std::string>(private_node_, "policy_sha256");
    const std::string epoch_text =
        requiredParam<std::string>(private_node_, "epoch_id");

    config_ = FrozenConfigLoader::loadFile(policy_file_, policy_sha256);
    expected_epoch_ = parseRosPrivateEpochId(epoch_text);
    epoch_fence_lease_ = std::make_unique<EpochFenceLease>(policy_file_);
    loadAndValidateEpochFence(policy_file_, config_, expected_epoch_);
    guard_ = std::make_unique<SessionClockGuard>(
        config_, expected_epoch_, steadyNowNs(), systemWallNowNs());
    loadAndValidateEpochFence(policy_file_, config_, expected_epoch_);

    const auto queue_depth = config_.thresholds.io_queue_depth;
    aggregate_publisher_ = node_.advertise<ClockGuardAggregateStatus>(
        "/xgc/session/clock/status", queue_depth, true);
    event_publisher_ = node_.advertise<ClockGuardEvent>(
        "/xgc/session/clock/events", queue_depth, false);
    for (const auto &route : config_.routes) {
      pose_publishers_.emplace(
          route.slot, node_.advertise<geometry_msgs::PoseStamped>(
                          canonicalPoseTopic(route), queue_depth, false));
      twist_publishers_.emplace(
          route.slot, node_.advertise<geometry_msgs::TwistStamped>(
                          canonicalTwistTopic(route), queue_depth, false));
      envelope_publishers_.emplace(
          route.slot, node_.advertise<ClockTimestampEnvelope>(
                          envelopeTopic(route), queue_depth, false));
      status_publishers_.emplace(
          route.slot, node_.advertise<ClockGuardStatus>(statusTopic(route),
                                                        queue_depth, true));

      pose_subscribers_.push_back(node_.subscribe<geometry_msgs::PoseStamped>(
          rawPoseTopic(route), queue_depth,
          [this, route](const geometry_msgs::PoseStamped::ConstPtr &message) {
            handlePose(route, message);
          }));
      twist_subscribers_.push_back(node_.subscribe<geometry_msgs::TwistStamped>(
          rawTwistTopic(route), queue_depth,
          [this, route](const geometry_msgs::TwistStamped::ConstPtr &message) {
            handleTwist(route, message);
          }));
    }

    const std::string clock_topic = gazeboClockTopic(config_);
    if (!clock_topic.empty()) {
      gazebo_clock_subscriber_ = node_.subscribe<rosgraph_msgs::Clock>(
          clock_topic, queue_depth, &SessionClockGuardNode::handleGazeboClock,
          this);
      ROS_WARN_STREAM(
          "Gazebo clock publisher identity/uniqueness admission for "
          << clock_topic
          << " is a Core workflow responsibility; this node enforces positive, "
             "monotonic, freshness, skew, and mode-specific RTF gates");
    }

    const double poll_period_seconds =
        static_cast<double>(config_.thresholds.guard_poll_period_ns) /
        1000000000.0;
    guard_poll_timer_ =
        node_.createWallTimer(ros::WallDuration(poll_period_seconds),
                              &SessionClockGuardNode::pollGuard, this);

    publishAllStatuses();
    ROS_INFO_STREAM("session clock guard initialized for Session "
                    << config_.session_id << " epoch=" << expected_epoch_
                    << " mode=" << config_.run_mode
                    << " authority=" << toString(config_.session_time_authority)
                    << " mapping=" << toString(config_.clock_mapping)
                    << " routes=" << config_.routes.size()
                    << " policy_revision=" << config_.policy_revision
                    << " policy_sha256=" << config_.policy_sha256);
  }

private:
  bool ensureEpochFence() {
    if (epoch_fence_failed_) {
      return false;
    }
    try {
      epoch_fence_lease_->validateCurrent();
      loadAndValidateEpochFence(policy_file_, config_, expected_epoch_);
      return true;
    } catch (const std::exception &error) {
      epoch_fence_failed_ = true;
      const std::string reason =
          std::string("immutable Core epoch fence failed: ") + error.what();
      guard_->failEpochFence(reason, steadyNowNs());
      publishAllStatuses();
      ROS_FATAL_STREAM(
          reason << "; canonical output is closed and the process will exit");
      ros::shutdown();
      return false;
    }
  }

  ros::Time sessionHeaderStamp() const {
    return rosTimeFromNs(guard_->sessionNow(steadyNowNs()));
  }

  Observation makeObservation(const Route &route, StreamKind stream,
                              std::uint64_t raw_stamp_ns) const {
    Observation observation;
    observation.slot = route.slot;
    observation.source_body = route.source_body;
    observation.source_domain = route.source_domain;
    observation.stream = stream;
    observation.raw_source_stamp_ns = raw_stamp_ns;
    observation.receive_monotonic_stamp_ns = steadyNowNs();
    observation.receive_system_wall_stamp_ns = systemWallNowNs();
    observation.fallback_source = false;
    return observation;
  }

  void handleGazeboClock(const rosgraph_msgs::Clock::ConstPtr &message) {
    if (!ensureEpochFence()) {
      return;
    }
    GazeboClockObservation observation;
    observation.gazebo_stamp_ns = message->clock.toNSec();
    observation.receive_monotonic_stamp_ns = steadyNowNs();
    observation.receive_system_wall_stamp_ns = systemWallNowNs();
    std::string reason;
    guard_->observeGazeboClock(observation, &reason);
    publishAllStatuses();
  }

  void handlePose(const Route &route,
                  const geometry_msgs::PoseStamped::ConstPtr &message) {
    if (!ensureEpochFence()) {
      return;
    }
    const auto result = guard_->observe(makeObservation(
        route, StreamKind::Pose, message->header.stamp.toNSec()));
    bool published = false;
    if (result.canonical_publish_allowed) {
      // Re-read the persisted Core epoch fence after mapping and immediately
      // before the sensitive publish. The callback-entry check protects all
      // processing; this second check narrows the epoch-advance race at the
      // actual canonical data-plane boundary.
      if (!ensureEpochFence()) {
        return;
      }
      auto output = *message;
      output.header.stamp = rosTimeFromNs(result.mapped_session_stamp_ns);
      pose_publishers_.at(route.slot).publish(output);
      published = true;
    }
    publishResult(result, published);
  }

  void handleTwist(const Route &route,
                   const geometry_msgs::TwistStamped::ConstPtr &message) {
    if (!ensureEpochFence()) {
      return;
    }
    const auto result = guard_->observe(makeObservation(
        route, StreamKind::Twist, message->header.stamp.toNSec()));
    bool published = false;
    if (result.canonical_publish_allowed) {
      if (!ensureEpochFence()) {
        return;
      }
      auto output = *message;
      output.header.stamp = rosTimeFromNs(result.mapped_session_stamp_ns);
      twist_publishers_.at(route.slot).publish(output);
      published = true;
    }
    publishResult(result, published);
  }

  void publishResult(const TimestampEnvelope &result,
                     bool canonical_published) {
    ClockTimestampEnvelope message;
    message.header.stamp = sessionHeaderStamp();
    message.session_id = config_.session_id;
    message.session_contract_sha256 = config_.session_contract_sha256;
    message.policy_sha256 = config_.policy_sha256;
    message.run_mode = config_.run_mode;
    message.session_time_authority = toString(config_.session_time_authority);
    message.clock_mapping = toString(config_.clock_mapping);
    message.slot = result.slot;
    message.source_body = result.source_body;
    message.canonical_body = result.canonical_body;
    message.source_domain = toString(result.source_domain);
    message.stream = toString(result.stream);
    message.epoch = result.epoch;
    message.raw_source_stamp = rosTimeFromNs(result.raw_source_stamp_ns);
    message.mapped_session_stamp =
        rosTimeFromNs(result.mapped_session_stamp_ns);
    message.receive_monotonic_stamp_ns = result.receive_monotonic_stamp_ns;
    message.authority_age_ns = result.authority_age_ns;
    message.gazebo_clock_skew_ns = result.gazebo_clock_skew_ns;
    message.offset_ns = result.offset_ns;
    message.drift_ppm = result.drift_ppm;
    message.jitter_ns = result.jitter_ns;
    message.uncertainty_ns = result.uncertainty_ns;
    message.state = static_cast<std::uint8_t>(result.state);
    message.accepted = result.accepted;
    message.canonical_published = canonical_published;
    message.reason = result.reason;
    envelope_publishers_.at(result.slot).publish(message);
    publishStatus(result.slot);
    publishAggregate();
    publishEvents();
  }

  void publishStatus(const std::string &slot) {
    const auto status = guard_->routeStatus(slot);
    ClockGuardStatus message;
    message.header.stamp = sessionHeaderStamp();
    message.session_id = config_.session_id;
    message.session_contract_sha256 = config_.session_contract_sha256;
    message.policy_sha256 = config_.policy_sha256;
    message.run_mode = config_.run_mode;
    message.session_time_authority = toString(config_.session_time_authority);
    message.clock_mapping = toString(config_.clock_mapping);
    message.slot = status.slot;
    message.source_body = status.source_body;
    message.canonical_body = status.canonical_body;
    message.source_domain = toString(status.source_domain);
    message.epoch = status.epoch;
    message.state = static_cast<std::uint8_t>(status.state);
    message.offset_ns = status.offset_ns;
    message.drift_ppm = status.drift_ppm;
    message.jitter_ns = status.jitter_ns;
    message.uncertainty_ns = status.uncertainty_ns;
    message.last_raw_source_stamp_ns = status.last_raw_source_stamp_ns;
    message.last_session_stamp_ns = status.last_session_stamp_ns;
    message.last_receive_monotonic_stamp_ns =
        status.last_receive_monotonic_stamp_ns;
    message.authority_age_ns = status.authority_age_ns;
    message.gazebo_clock_skew_ns = status.gazebo_clock_skew_ns;
    message.station_wall_error_ns = status.station_wall_error_ns;
    message.station_wall_step_ns = status.station_wall_step_ns;
    message.gazebo_real_time_factor = status.gazebo_real_time_factor;
    message.healthy_samples = status.healthy_samples;
    message.consecutive_failures = status.consecutive_failures;
    message.reason = status.reason;
    status_publishers_.at(slot).publish(message);
  }

  void publishAggregate() {
    const auto authority = guard_->authorityStatus();
    ClockGuardAggregateStatus message;
    message.header.stamp = sessionHeaderStamp();
    message.session_id = config_.session_id;
    message.session_contract_sha256 = config_.session_contract_sha256;
    message.policy_sha256 = config_.policy_sha256;
    message.run_mode = config_.run_mode;
    message.session_time_authority = toString(authority.authority);
    message.clock_mapping = toString(authority.mapping);
    message.status_sequence = next_status_sequence_++;
    message.epoch = guard_->epoch();
    message.state = static_cast<std::uint8_t>(guard_->aggregateState());
    message.sensitive_output_allowed = guard_->sensitiveOutputAllowed();
    message.authority_state = static_cast<std::uint8_t>(authority.state);
    message.authority_healthy = authority.healthy;
    message.gazebo_clock_required = authority.gazebo_clock_required;
    message.gazebo_clock_ready = authority.gazebo_clock_ready;
    message.gazebo_clock_topic = gazeboClockTopic(config_);
    message.session_now_ns = authority.session_now_ns;
    message.last_authority_monotonic_stamp_ns =
        authority.last_authority_monotonic_stamp_ns;
    message.authority_age_ns = authority.authority_age_ns;
    message.last_gazebo_clock_stamp_ns = authority.last_gazebo_clock_stamp_ns;
    message.last_gazebo_clock_skew_ns = authority.last_gazebo_clock_skew_ns;
    message.station_wall_error_ns = authority.station_wall_error_ns;
    message.station_wall_step_ns = authority.station_wall_step_ns;
    message.gazebo_real_time_factor = authority.gazebo_real_time_factor;
    message.authority_consecutive_failures = authority.consecutive_failures;
    message.authority_recovery_samples = authority.recovery_samples;
    message.required_routes = static_cast<std::uint32_t>(config_.routes.size());
    message.locked_routes =
        static_cast<std::uint32_t>(guard_->lockedRouteCount());
    message.vrpn_wire_resolution_ns = config_.vrpn_wire_resolution_ns;
    message.measurement_delay_enabled = config_.measurement_delay_enabled;
    message.delay_timestamp_policy = config_.delay_timestamp_policy;
    message.reason = authority.reason + "; aggregate clock guard state is " +
                     toString(guard_->aggregateState());
    aggregate_publisher_.publish(message);
  }

  void publishEvents() {
    for (const auto &event : guard_->takeEvents()) {
      ClockGuardEvent message;
      message.header.stamp = rosTimeFromNs(event.session_stamp_ns);
      message.session_id = config_.session_id;
      message.session_contract_sha256 = config_.session_contract_sha256;
      message.policy_sha256 = config_.policy_sha256;
      message.run_mode = config_.run_mode;
      message.session_time_authority = toString(config_.session_time_authority);
      message.clock_mapping = toString(config_.clock_mapping);
      message.sequence = event.sequence;
      message.event = static_cast<std::uint8_t>(event.kind);
      message.state = static_cast<std::uint8_t>(event.state);
      message.epoch = event.epoch;
      message.monotonic_stamp_ns = event.monotonic_stamp_ns;
      message.session_stamp_ns = event.session_stamp_ns;
      message.authority_age_ns = event.authority_age_ns;
      message.last_gazebo_clock_stamp_ns = event.last_gazebo_clock_stamp_ns;
      message.gazebo_clock_skew_ns = event.gazebo_clock_skew_ns;
      message.station_wall_error_ns = event.station_wall_error_ns;
      message.station_wall_step_ns = event.station_wall_step_ns;
      message.gazebo_real_time_factor = event.gazebo_real_time_factor;
      message.reason = event.reason;
      event_publisher_.publish(message);
    }
  }

  void publishAllStatuses() {
    for (const auto &route : config_.routes) {
      publishStatus(route.slot);
    }
    publishAggregate();
    publishEvents();
  }

  void pollGuard(const ros::WallTimerEvent &) {
    if (!ensureEpochFence()) {
      return;
    }
    guard_->poll(steadyNowNs(), systemWallNowNs());
    publishAllStatuses();
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  FrozenConfig config_;
  std::unique_ptr<SessionClockGuard> guard_;
  std::unique_ptr<EpochFenceLease> epoch_fence_lease_;
  std::string policy_file_;
  std::uint64_t expected_epoch_{0U};
  bool epoch_fence_failed_{false};
  ros::Publisher aggregate_publisher_;
  ros::Publisher event_publisher_;
  ros::WallTimer guard_poll_timer_;
  ros::Subscriber gazebo_clock_subscriber_;
  std::map<std::string, ros::Publisher> pose_publishers_;
  std::map<std::string, ros::Publisher> twist_publishers_;
  std::map<std::string, ros::Publisher> envelope_publishers_;
  std::map<std::string, ros::Publisher> status_publishers_;
  std::vector<ros::Subscriber> pose_subscribers_;
  std::vector<ros::Subscriber> twist_subscribers_;
  std::uint64_t next_status_sequence_{1U};
};

} // namespace xgc_session_clock_guard

int main(int argc, char **argv) {
  ros::init(argc, argv, "xgc_session_clock_guard");
  try {
    xgc_session_clock_guard::SessionClockGuardNode node;
    ros::spin();
    return 0;
  } catch (const std::exception &error) {
    ROS_FATAL_STREAM(
        "session clock guard startup failed closed: " << error.what());
    return 2;
  }
}
