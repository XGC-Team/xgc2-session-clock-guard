#include "xgc_session_clock_guard/frozen_config.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <map>
#include <set>
#include <sstream>
#include <utility>

namespace xgc_session_clock_guard {
namespace {

constexpr const char* kSchema = "xgc.session-clock-guard.config.v2";
constexpr std::size_t kMaximumCanonicalPolicyBytes = 1U << 20U;
constexpr std::size_t kMaximumSessionIdBytes = 64U;
constexpr std::size_t kMaximumRouteIdentityBytes = 128U;
constexpr std::uint64_t kMaximumGuardPollPeriodNs = 250000000ULL;
constexpr std::uint64_t kMinimumStartupLockTimeoutNs = 250000000ULL;
constexpr std::uint64_t kMaximumStartupLockTimeoutNs = 3000000000ULL;

bool asciiAlpha(unsigned char value) {
  return (value >= 'A' && value <= 'Z') ||
         (value >= 'a' && value <= 'z');
}

bool asciiDigit(unsigned char value) {
  return value >= '0' && value <= '9';
}

bool asciiAlphaNumeric(unsigned char value) {
  return asciiAlpha(value) || asciiDigit(value);
}

bool isLowerHexDigest(const std::string& value) {
  return value.size() == 64U &&
         std::all_of(value.begin(), value.end(), [](unsigned char c) {
           return asciiDigit(c) || (c >= 'a' && c <= 'f');
         });
}

bool isRosSegment(const std::string& value) {
  if (value.empty() || value.size() > kMaximumRouteIdentityBytes ||
      !(asciiAlpha(static_cast<unsigned char>(value.front())) ||
        value.front() == '_')) {
    return false;
  }
  return std::all_of(value.begin() + 1, value.end(), [](unsigned char c) {
    return asciiAlphaNumeric(c) || c == '_';
  });
}

bool isSlotId(const std::string& value) {
  if (value.empty() || value.size() > kMaximumRouteIdentityBytes ||
      !asciiAlphaNumeric(static_cast<unsigned char>(value.front()))) {
    return false;
  }
  return std::all_of(value.begin() + 1, value.end(), [](unsigned char c) {
    return asciiAlphaNumeric(c) || c == '_' || c == '-';
  });
}

bool isSessionId(const std::string& value) {
  return !value.empty() && value.size() <= kMaximumSessionIdBytes &&
         std::all_of(value.begin(), value.end(), [](unsigned char c) {
           return asciiAlphaNumeric(c) || c == '-' || c == '_' || c == '.' ||
                  c == ':';
         });
}

std::uint64_t parseUnsigned(const std::string& key, const std::string& value) {
  if (value.empty() || (value.size() > 1U && value.front() == '0') ||
      !std::all_of(value.begin(), value.end(), [](unsigned char c) {
        return asciiDigit(c);
      })) {
    throw ConfigError(key + " must be a canonical unsigned decimal integer");
  }
  std::size_t consumed = 0;
  try {
    const auto parsed = std::stoull(value, &consumed, 10);
    if (consumed != value.size()) {
      throw ConfigError(key + " contains trailing characters");
    }
    return parsed;
  } catch (const std::invalid_argument&) {
    throw ConfigError(key + " is not an integer");
  } catch (const std::out_of_range&) {
    throw ConfigError(key + " is out of range");
  }
}

std::uint32_t parseUint32(const std::string& key, const std::string& value) {
  const auto parsed = parseUnsigned(key, value);
  if (parsed > std::numeric_limits<std::uint32_t>::max()) {
    throw ConfigError(key + " is out of uint32 range");
  }
  return static_cast<std::uint32_t>(parsed);
}

std::string fixedFromGeneral(std::string value) {
  const auto exponent_position = value.find_first_of("eE");
  if (exponent_position == std::string::npos) {
    return value;
  }
  const std::string mantissa = value.substr(0U, exponent_position);
  const int exponent = std::stoi(value.substr(exponent_position + 1U));
  const auto decimal_position = mantissa.find('.');
  std::string digits = mantissa;
  if (decimal_position != std::string::npos) {
    digits.erase(decimal_position, 1U);
  }
  const long long original_decimal =
      decimal_position == std::string::npos
          ? static_cast<long long>(mantissa.size())
          : static_cast<long long>(decimal_position);
  const long long target_decimal = original_decimal + exponent;
  if (target_decimal <= 0) {
    return "0." + std::string(static_cast<std::size_t>(-target_decimal), '0') +
           digits;
  }
  if (target_decimal >= static_cast<long long>(digits.size())) {
    return digits +
           std::string(static_cast<std::size_t>(target_decimal -
                                                static_cast<long long>(digits.size())),
                       '0');
  }
  digits.insert(static_cast<std::size_t>(target_decimal), 1U, '.');
  return digits;
}

double parseCLocaleDouble(const std::string& value, bool* complete) {
  char* end = nullptr;
  const double parsed = std::strtod(value.c_str(), &end);
  if (complete != nullptr) {
    *complete = end == value.c_str() + value.size();
  }
  return parsed;
}

bool sameDouble(double left, double right) {
  static_assert(sizeof(double) == sizeof(std::uint64_t),
                "Clock Guard requires IEEE-754 binary64");
  std::uint64_t left_bits = 0U;
  std::uint64_t right_bits = 0U;
  std::memcpy(&left_bits, &left, sizeof(left));
  std::memcpy(&right_bits, &right, sizeof(right));
  return left_bits == right_bits;
}

std::string canonicalFloating(double value) {
  for (int precision = 1;
       precision <= std::numeric_limits<double>::max_digits10; ++precision) {
    std::ostringstream stream;
    stream.imbue(std::locale::classic());
    stream << std::setprecision(precision) << std::defaultfloat << value;
    const std::string candidate = fixedFromGeneral(stream.str());
    bool complete = false;
    const double reparsed = parseCLocaleDouble(candidate, &complete);
    if (complete && sameDouble(reparsed, value)) {
      return candidate;
    }
  }
  throw ConfigError("cannot render a canonical positive float64");
}

double parsePositiveDouble(const std::string& key, const std::string& value) {
  if (value.empty() || value.front() == '+' || value.front() == '-' ||
      value.find_first_not_of("0123456789.") != std::string::npos ||
      std::count(value.begin(), value.end(), '.') > 1) {
    throw ConfigError(key + " must use canonical fixed-decimal float64 spelling");
  }
  bool complete = false;
  const double parsed = parseCLocaleDouble(value, &complete);
  if (!complete || !std::isfinite(parsed) || parsed <= 0.0 ||
      canonicalFloating(parsed) != value) {
    throw ConfigError(key + " must be a canonical positive finite float64");
  }
  return parsed;
}

bool parseBoolean(const std::string& key, const std::string& value) {
  if (value == "true") {
    return true;
  }
  if (value == "false") {
    return false;
  }
  throw ConfigError(key + " must be exactly true or false");
}

const std::string& require(const std::map<std::string, std::string>& values,
                           const std::string& key) {
  const auto it = values.find(key);
  if (it == values.end() || it->second.empty()) {
    throw ConfigError("missing required key: " + key);
  }
  return it->second;
}

template <typename Value>
bool present(const std::optional<Value>& value) {
  return value.has_value();
}

std::vector<std::string> canonicalKeyOrder(const FrozenConfig& config) {
  std::vector<std::string> keys{
      "schema",
      "session_id",
      "session_contract_sha256",
      "run_mode",
      "session_clock.authority",
      "session_clock.mapping",
      "vrpn.wire_time_resolution_ns",
      "delay.measurement_enabled",
      "delay.timestamp_policy",
  };
  const auto& policy = config.thresholds;
  if (policy.estimator_window.has_value()) {
    keys.emplace_back("threshold.estimator_window");
  }
  keys.emplace_back("threshold.min_lock_samples");
  keys.emplace_back("threshold.recover_lock_samples");
  keys.emplace_back("threshold.lost_after_failures");
  keys.emplace_back("threshold.startup_lock_timeout_ns");
  if (policy.max_offset_step_ns.has_value()) {
    keys.emplace_back("threshold.max_offset_step_ns");
  }
  if (policy.max_drift_ppm.has_value()) {
    keys.emplace_back("threshold.max_drift_ppm");
  }
  if (policy.max_jitter_ns.has_value()) {
    keys.emplace_back("threshold.max_jitter_ns");
  }
  keys.emplace_back("threshold.max_uncertainty_ns");
  keys.emplace_back("threshold.max_sample_age_ns");
  keys.emplace_back("threshold.max_authority_age_ns");
  if (policy.max_gazebo_clock_skew_ns.has_value()) {
    keys.emplace_back("threshold.max_gazebo_clock_skew_ns");
  }
  if (policy.max_station_wall_error_ns.has_value()) {
    keys.emplace_back("threshold.max_station_wall_error_ns");
  }
  if (policy.max_station_wall_step_ns.has_value()) {
    keys.emplace_back("threshold.max_station_wall_step_ns");
  }
  if (policy.min_gazebo_real_time_factor.has_value()) {
    keys.emplace_back("threshold.min_gazebo_real_time_factor");
  }
  if (policy.max_gazebo_real_time_factor.has_value()) {
    keys.emplace_back("threshold.max_gazebo_real_time_factor");
  }
  keys.emplace_back("threshold.guard_poll_period_ns");
  keys.emplace_back("io.queue_depth");
  for (const auto& route : config.routes) {
    const std::string prefix = "route." + route.slot + ".";
    keys.push_back(prefix + "source_domain");
    keys.push_back(prefix + "source_body");
    keys.push_back(prefix + "canonical_body");
    keys.push_back(prefix + "sample_period_ns");
    keys.push_back(prefix + "source_uncertainty_ns");
  }
  return keys;
}

}  // namespace

void validateThresholdPolicy(const ThresholdPolicy& policy) {
  if (policy.min_lock_samples < 2U || policy.min_lock_samples > 4096U) {
    throw ConfigError("threshold.min_lock_samples must be in [2, 4096]");
  }
  if (policy.recover_lock_samples == 0U || policy.recover_lock_samples > 4096U) {
    throw ConfigError("threshold.recover_lock_samples must be in [1, 4096]");
  }
  if (policy.estimator_window.has_value()) {
    if (*policy.estimator_window < 2U || *policy.estimator_window > 4096U) {
      throw ConfigError("threshold.estimator_window must be in [2, 4096]");
    }
    if (policy.min_lock_samples > *policy.estimator_window ||
        policy.recover_lock_samples > *policy.estimator_window) {
      throw ConfigError(
          "lock and recovery sample counts cannot exceed threshold.estimator_window");
    }
  }
  if (policy.lost_after_failures == 0U ||
      policy.startup_lock_timeout_ns == 0U ||
      policy.max_uncertainty_ns == 0U ||
      policy.max_sample_age_ns == 0U || policy.max_authority_age_ns == 0U ||
      policy.guard_poll_period_ns == 0U || policy.io_queue_depth == 0U) {
    throw ConfigError("all common clock and IO thresholds must be greater than zero");
  }
  if (policy.guard_poll_period_ns > policy.max_sample_age_ns ||
      policy.guard_poll_period_ns > policy.max_authority_age_ns) {
    throw ConfigError(
        "threshold.guard_poll_period_ns cannot exceed sample or authority age limits");
  }
  if (policy.guard_poll_period_ns > kMaximumGuardPollPeriodNs) {
    throw ConfigError(
        "threshold.guard_poll_period_ns exceeds the fixed 250000000 ns readiness bound");
  }
  if (policy.startup_lock_timeout_ns < kMinimumStartupLockTimeoutNs ||
      policy.startup_lock_timeout_ns > kMaximumStartupLockTimeoutNs) {
    throw ConfigError(
        "threshold.startup_lock_timeout_ns must be in [250000000, 3000000000]");
  }
  if (policy.startup_lock_timeout_ns < policy.max_sample_age_ns ||
      policy.startup_lock_timeout_ns < policy.max_authority_age_ns) {
    throw ConfigError(
        "threshold.startup_lock_timeout_ns cannot be below runtime freshness bounds");
  }
  if (policy.io_queue_depth > 65536U) {
    throw ConfigError("io.queue_depth exceeds the bounded schema maximum of 65536");
  }
  if (policy.max_offset_step_ns.has_value() && *policy.max_offset_step_ns == 0U) {
    throw ConfigError("threshold.max_offset_step_ns must be greater than zero");
  }
  if (policy.max_drift_ppm.has_value() &&
      (!std::isfinite(*policy.max_drift_ppm) || *policy.max_drift_ppm <= 0.0)) {
    throw ConfigError("threshold.max_drift_ppm must be finite and greater than zero");
  }
  if (policy.max_jitter_ns.has_value()) {
    if (*policy.max_jitter_ns == 0U) {
      throw ConfigError("threshold.max_jitter_ns must be greater than zero");
    }
    if (*policy.max_jitter_ns > policy.max_uncertainty_ns) {
      throw ConfigError("max_jitter_ns cannot exceed max_uncertainty_ns");
    }
  }
  for (const auto* threshold : {&policy.max_gazebo_clock_skew_ns,
                                &policy.max_station_wall_error_ns,
                                &policy.max_station_wall_step_ns}) {
    if (threshold->has_value() && **threshold == 0U) {
      throw ConfigError("optional clock thresholds must be greater than zero when present");
    }
  }
  if (policy.min_gazebo_real_time_factor.has_value() &&
      (!std::isfinite(*policy.min_gazebo_real_time_factor) ||
       *policy.min_gazebo_real_time_factor <= 0.0)) {
    throw ConfigError("threshold.min_gazebo_real_time_factor must be positive and finite");
  }
  if (policy.max_gazebo_real_time_factor.has_value() &&
      (!std::isfinite(*policy.max_gazebo_real_time_factor) ||
       *policy.max_gazebo_real_time_factor <= 0.0)) {
    throw ConfigError("threshold.max_gazebo_real_time_factor must be positive and finite");
  }
}

void validateRoutes(const std::string& run_mode, const std::vector<Route>& routes) {
  if (run_mode != "simulation" && run_mode != "physical" && run_mode != "hybrid") {
    throw ConfigError("run_mode must be exactly simulation, physical, or hybrid");
  }
  if (routes.empty()) {
    if (run_mode == "hybrid") {
      throw ConfigError("hybrid run must contain both simulation and physical routes");
    }
    return;
  }
  std::set<std::string> slots;
  std::set<std::string> canonical_bodies;
  std::set<std::string> raw_sources;
  bool has_simulation = false;
  bool has_physical = false;
  for (const auto& route : routes) {
    if (!isSlotId(route.slot) || !isRosSegment(route.source_body) ||
        !isRosSegment(route.canonical_body)) {
      throw ConfigError(
          "route slot must be a stable SlotID and both body fields must be legal single ROS name segments");
    }
    if (!slots.insert(route.slot).second) {
      throw ConfigError("duplicate route slot: " + route.slot);
    }
    if (!canonical_bodies.insert(route.canonical_body).second) {
      throw ConfigError("duplicate canonical body: " + route.canonical_body);
    }
    const std::string raw_source = rawRoot(route.source_domain) + "/" + route.source_body;
    if (!raw_sources.insert(raw_source).second) {
      throw ConfigError("duplicate raw VRPN source: " + raw_source);
    }
    if (route.sample_period_ns == 0U || route.source_uncertainty_ns == 0U) {
      throw ConfigError(
          "route sample_period_ns and source_uncertainty_ns must be greater than zero");
    }
    has_simulation = has_simulation || route.source_domain == SourceDomain::Simulation;
    has_physical = has_physical || route.source_domain == SourceDomain::Physical;
    if (run_mode == "simulation" && route.source_domain != SourceDomain::Simulation) {
      throw ConfigError("simulation run contains a physical route");
    }
    if (run_mode == "physical" && route.source_domain != SourceDomain::Physical) {
      throw ConfigError("physical run contains a simulation route");
    }
  }
  if (run_mode == "hybrid" && (!has_simulation || !has_physical)) {
    throw ConfigError("hybrid run must contain both simulation and physical routes");
  }
}

void validateModeClockPolicy(const FrozenConfig& config) {
  const auto& policy = config.thresholds;
  const bool affine_fields = present(policy.estimator_window) &&
                             present(policy.max_offset_step_ns) &&
                             present(policy.max_drift_ppm) &&
                             present(policy.max_jitter_ns);
  const bool any_affine_field = present(policy.estimator_window) ||
                                present(policy.max_offset_step_ns) ||
                                present(policy.max_drift_ppm) ||
                                present(policy.max_jitter_ns);
  const bool station_fields = present(policy.max_station_wall_error_ns) &&
                              present(policy.max_station_wall_step_ns);
  const bool any_station_field = present(policy.max_station_wall_error_ns) ||
                                 present(policy.max_station_wall_step_ns);
  const bool rtf_fields = present(policy.min_gazebo_real_time_factor) &&
                          present(policy.max_gazebo_real_time_factor);
  const bool any_rtf_field = present(policy.min_gazebo_real_time_factor) ||
                             present(policy.max_gazebo_real_time_factor);

  if (config.run_mode == "simulation") {
    if (config.session_time_authority != SessionTimeAuthority::GazeboSimulation ||
        config.clock_mapping != ClockMapping::Identity) {
      throw ConfigError(
          "simulation requires gazebo-simulation authority with identity mapping");
    }
    if (!present(policy.max_gazebo_clock_skew_ns)) {
      throw ConfigError("simulation requires threshold.max_gazebo_clock_skew_ns");
    }
    if (any_affine_field || any_station_field || any_rtf_field) {
      throw ConfigError(
          "simulation forbids affine, station-wall, and Gazebo real-time-factor thresholds");
    }
    return;
  }

  if (config.session_time_authority != SessionTimeAuthority::StationWallMonotonic ||
      config.clock_mapping != ClockMapping::AffineToSession) {
    throw ConfigError(
        "physical and hybrid require station-wall-monotonic authority with affine-to-session mapping");
  }
  if (!affine_fields || !station_fields) {
    throw ConfigError(
        "physical and hybrid require the complete affine and station-wall threshold sets");
  }
  if (config.run_mode == "physical") {
    if (present(policy.max_gazebo_clock_skew_ns) || any_rtf_field) {
      throw ConfigError("physical forbids Gazebo skew and real-time-factor thresholds");
    }
    return;
  }

  if (!present(policy.max_gazebo_clock_skew_ns) || !rtf_fields) {
    throw ConfigError(
        "hybrid requires Gazebo skew plus minimum and maximum real-time-factor thresholds");
  }
  if (*policy.min_gazebo_real_time_factor > 1.0 ||
      *policy.max_gazebo_real_time_factor < 1.0 ||
      *policy.min_gazebo_real_time_factor >= *policy.max_gazebo_real_time_factor) {
    throw ConfigError(
        "hybrid Gazebo real-time-factor bounds must strictly bracket 1.0");
  }
}

void validateSourceTiming(const FrozenConfig& config) {
  constexpr std::uint64_t kGazeboVrpnV24WireResolutionNs = 1000U;
  if (config.vrpn_wire_resolution_ns != kGazeboVrpnV24WireResolutionNs) {
    throw ConfigError(
        "vrpn.wire_time_resolution_ns must be exactly 1000 for the v24 timeval contract");
  }
  if (config.delay_timestamp_policy != "sample_time") {
    throw ConfigError(
        "delay.timestamp_policy must be exactly sample_time; send_time is forbidden");
  }
  if (config.clock_mapping == ClockMapping::AffineToSession &&
      (*config.thresholds.max_offset_step_ns < config.vrpn_wire_resolution_ns ||
       *config.thresholds.max_jitter_ns < config.vrpn_wire_resolution_ns)) {
    throw ConfigError("affine offset-step and jitter thresholds cannot be below VRPN wire quantization");
  }
  if (config.thresholds.max_gazebo_clock_skew_ns.has_value() &&
      *config.thresholds.max_gazebo_clock_skew_ns < config.vrpn_wire_resolution_ns) {
    throw ConfigError("Gazebo clock skew threshold cannot be below VRPN wire quantization");
  }
  if (config.thresholds.max_station_wall_step_ns.has_value() &&
      *config.thresholds.max_station_wall_step_ns >
          *config.thresholds.max_station_wall_error_ns) {
    throw ConfigError("station wall step threshold cannot exceed the wall error threshold");
  }
  for (const auto& route : config.routes) {
    const std::uint64_t half_sample_period =
        route.sample_period_ns / 2U + route.sample_period_ns % 2U;
    if (half_sample_period > std::numeric_limits<std::uint64_t>::max() -
                                 config.vrpn_wire_resolution_ns) {
      throw ConfigError("route timing uncertainty floor overflows uint64");
    }
    const std::uint64_t uncertainty_floor =
        config.vrpn_wire_resolution_ns + half_sample_period;
    if (route.source_uncertainty_ns < uncertainty_floor) {
      throw ConfigError("route." + route.slot +
                        ".source_uncertainty_ns is below wire quantization plus half-sample floor");
    }
    if (config.thresholds.max_uncertainty_ns < route.source_uncertainty_ns) {
      throw ConfigError("route." + route.slot +
                        " source uncertainty exceeds max_uncertainty_ns");
    }
    if (config.thresholds.max_sample_age_ns < route.sample_period_ns) {
      throw ConfigError("max_sample_age_ns cannot be below route." + route.slot +
                        " sample_period_ns");
    }
  }
}

FrozenConfig FrozenConfigLoader::loadFile(const std::string& path,
                                          const std::string& expected_sha256) {
  if (path.empty()) {
    throw ConfigError("policy_file is required");
  }
  if (!isLowerHexDigest(expected_sha256)) {
    throw ConfigError("policy_sha256 must be exactly 64 lowercase hex characters");
  }
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    throw ConfigError("cannot open frozen config: " + path);
  }
  stream.seekg(0, std::ios::end);
  const auto size = stream.tellg();
  if (size <= 0 ||
      static_cast<std::uint64_t>(size) > kMaximumCanonicalPolicyBytes) {
    throw ConfigError("frozen config must be in (0, 1 MiB]");
  }
  stream.seekg(0, std::ios::beg);
  std::string content(static_cast<std::size_t>(size), '\0');
  stream.read(&content.front(), size);
  if (!stream || stream.gcount() != size) {
    throw ConfigError("cannot read frozen config: " + path);
  }
  const std::string actual_sha256 = sha256Hex(content);
  if (actual_sha256 != expected_sha256) {
    throw ConfigError("frozen config sha256 mismatch: expected " + expected_sha256 +
                      ", got " + actual_sha256);
  }
  return parse(content, actual_sha256);
}

FrozenConfig FrozenConfigLoader::parse(const std::string& bytes,
                                       const std::string& verified_sha256) {
  if (!isLowerHexDigest(verified_sha256)) {
    throw ConfigError("verified config digest is invalid");
  }
  if (bytes.empty() || bytes.size() > kMaximumCanonicalPolicyBytes ||
      bytes.back() != '\n' || bytes.find('\r') != std::string::npos ||
      bytes.find("\n\n") != std::string::npos) {
    throw ConfigError(
        "canonical config must be in (0, 1 MiB], contain no CR, and end in one LF");
  }

  std::map<std::string, std::string> values;
  std::vector<std::string> ordered_keys;
  std::istringstream lines(bytes.substr(0U, bytes.size() - 1U));
  std::string line;
  std::size_t line_number = 0U;
  while (std::getline(lines, line)) {
    ++line_number;
    if (line.empty() || line.front() == '#' ||
        std::any_of(line.begin(), line.end(), [](unsigned char character) {
          return character == ' ' || character == '\t' ||
                 character == '\v' || character == '\f';
        })) {
      throw ConfigError("line " + std::to_string(line_number) +
                        " is not canonical key=value data");
    }
    const auto separator = line.find('=');
    if (separator == std::string::npos ||
        line.find('=', separator + 1U) != std::string::npos) {
      throw ConfigError("line " + std::to_string(line_number) +
                        " must contain exactly one '='");
    }
    const std::string key = line.substr(0, separator);
    const std::string value = line.substr(separator + 1U);
    if (key.empty() || value.empty()) {
      throw ConfigError("line " + std::to_string(line_number) +
                        " has an empty key or value");
    }
    if (!values.emplace(key, value).second) {
      throw ConfigError("duplicate config key: " + key);
    }
    ordered_keys.push_back(key);
  }

  const std::set<std::string> scalar_keys{
      "schema",
      "session_id",
      "session_contract_sha256",
      "run_mode",
      "session_clock.authority",
      "session_clock.mapping",
      "vrpn.wire_time_resolution_ns",
      "delay.measurement_enabled",
      "delay.timestamp_policy",
      "threshold.estimator_window",
      "threshold.min_lock_samples",
      "threshold.recover_lock_samples",
      "threshold.lost_after_failures",
      "threshold.startup_lock_timeout_ns",
      "threshold.max_offset_step_ns",
      "threshold.max_drift_ppm",
      "threshold.max_jitter_ns",
      "threshold.max_uncertainty_ns",
      "threshold.max_sample_age_ns",
      "threshold.max_authority_age_ns",
      "threshold.max_gazebo_clock_skew_ns",
      "threshold.max_station_wall_error_ns",
      "threshold.max_station_wall_step_ns",
      "threshold.min_gazebo_real_time_factor",
      "threshold.max_gazebo_real_time_factor",
      "threshold.guard_poll_period_ns",
      "io.queue_depth",
  };

  struct RouteFields {
    std::map<std::string, std::string> fields;
  };
  std::map<std::string, RouteFields> route_fields;
  for (const auto& item : values) {
    if (scalar_keys.count(item.first) != 0U) {
      continue;
    }
    constexpr const char* prefix = "route.";
    if (item.first.compare(0, 6, prefix) != 0) {
      throw ConfigError("unknown config key: " + item.first);
    }
    const auto field_separator = item.first.find('.', 6U);
    if (field_separator == std::string::npos) {
      throw ConfigError("route key must be route.<slot>.<field>: " + item.first);
    }
    const std::string slot = item.first.substr(6U, field_separator - 6U);
    const std::string field = item.first.substr(field_separator + 1U);
    if (!isSlotId(slot) ||
        (field != "source_domain" && field != "source_body" &&
         field != "canonical_body" && field != "sample_period_ns" &&
         field != "source_uncertainty_ns")) {
      throw ConfigError("invalid route key: " + item.first);
    }
    route_fields[slot].fields.emplace(field, item.second);
  }

  FrozenConfig config;
  config.schema = require(values, "schema");
  if (config.schema != kSchema) {
    throw ConfigError("schema must be exactly " + std::string(kSchema));
  }
  config.session_id = require(values, "session_id");
  if (!isSessionId(config.session_id)) {
    throw ConfigError("session_id contains invalid characters");
  }
  config.session_contract_sha256 = require(values, "session_contract_sha256");
  if (!isLowerHexDigest(config.session_contract_sha256)) {
    throw ConfigError("session_contract_sha256 must be 64 lowercase hex characters");
  }
  config.run_mode = require(values, "run_mode");
  try {
    config.session_time_authority =
        parseSessionTimeAuthority(require(values, "session_clock.authority"));
    config.clock_mapping = parseClockMapping(require(values, "session_clock.mapping"));
  } catch (const std::invalid_argument& error) {
    throw ConfigError(error.what());
  }
  config.policy_sha256 = verified_sha256;
  config.vrpn_wire_resolution_ns = parseUnsigned(
      "vrpn.wire_time_resolution_ns", require(values, "vrpn.wire_time_resolution_ns"));
  config.measurement_delay_enabled = parseBoolean(
      "delay.measurement_enabled", require(values, "delay.measurement_enabled"));
  config.delay_timestamp_policy = require(values, "delay.timestamp_policy");

  auto& policy = config.thresholds;
  if (values.count("threshold.estimator_window") != 0U) {
    policy.estimator_window = static_cast<std::size_t>(parseUnsigned(
        "threshold.estimator_window", values.at("threshold.estimator_window")));
  }
  policy.min_lock_samples = parseUint32(
      "threshold.min_lock_samples", require(values, "threshold.min_lock_samples"));
  policy.recover_lock_samples = parseUint32(
      "threshold.recover_lock_samples", require(values, "threshold.recover_lock_samples"));
  policy.lost_after_failures = parseUint32(
      "threshold.lost_after_failures", require(values, "threshold.lost_after_failures"));
  policy.startup_lock_timeout_ns = parseUnsigned(
      "threshold.startup_lock_timeout_ns",
      require(values, "threshold.startup_lock_timeout_ns"));
  if (values.count("threshold.max_offset_step_ns") != 0U) {
    policy.max_offset_step_ns = parseUnsigned(
        "threshold.max_offset_step_ns", values.at("threshold.max_offset_step_ns"));
  }
  if (values.count("threshold.max_drift_ppm") != 0U) {
    policy.max_drift_ppm = parsePositiveDouble(
        "threshold.max_drift_ppm", values.at("threshold.max_drift_ppm"));
  }
  if (values.count("threshold.max_jitter_ns") != 0U) {
    policy.max_jitter_ns = parseUnsigned(
        "threshold.max_jitter_ns", values.at("threshold.max_jitter_ns"));
  }
  policy.max_uncertainty_ns = parseUnsigned(
      "threshold.max_uncertainty_ns", require(values, "threshold.max_uncertainty_ns"));
  policy.max_sample_age_ns = parseUnsigned(
      "threshold.max_sample_age_ns", require(values, "threshold.max_sample_age_ns"));
  policy.max_authority_age_ns = parseUnsigned(
      "threshold.max_authority_age_ns", require(values, "threshold.max_authority_age_ns"));
  if (values.count("threshold.max_gazebo_clock_skew_ns") != 0U) {
    policy.max_gazebo_clock_skew_ns = parseUnsigned(
        "threshold.max_gazebo_clock_skew_ns",
        values.at("threshold.max_gazebo_clock_skew_ns"));
  }
  if (values.count("threshold.max_station_wall_error_ns") != 0U) {
    policy.max_station_wall_error_ns = parseUnsigned(
        "threshold.max_station_wall_error_ns",
        values.at("threshold.max_station_wall_error_ns"));
  }
  if (values.count("threshold.max_station_wall_step_ns") != 0U) {
    policy.max_station_wall_step_ns = parseUnsigned(
        "threshold.max_station_wall_step_ns",
        values.at("threshold.max_station_wall_step_ns"));
  }
  if (values.count("threshold.min_gazebo_real_time_factor") != 0U) {
    policy.min_gazebo_real_time_factor = parsePositiveDouble(
        "threshold.min_gazebo_real_time_factor",
        values.at("threshold.min_gazebo_real_time_factor"));
  }
  if (values.count("threshold.max_gazebo_real_time_factor") != 0U) {
    policy.max_gazebo_real_time_factor = parsePositiveDouble(
        "threshold.max_gazebo_real_time_factor",
        values.at("threshold.max_gazebo_real_time_factor"));
  }
  policy.guard_poll_period_ns = parseUnsigned(
      "threshold.guard_poll_period_ns", require(values, "threshold.guard_poll_period_ns"));
  policy.io_queue_depth =
      parseUint32("io.queue_depth", require(values, "io.queue_depth"));

  validateThresholdPolicy(policy);

  for (const auto& route_item : route_fields) {
    const auto& fields = route_item.second.fields;
    if (fields.size() != 5U || fields.count("source_domain") == 0U ||
        fields.count("source_body") == 0U || fields.count("canonical_body") == 0U ||
        fields.count("sample_period_ns") == 0U ||
        fields.count("source_uncertainty_ns") == 0U) {
      throw ConfigError(
          "route." + route_item.first +
          " requires exactly source_domain, source_body, canonical_body, sample_period_ns, and source_uncertainty_ns");
    }
    Route route;
    route.slot = route_item.first;
    try {
      route.source_domain = parseSourceDomain(fields.at("source_domain"));
    } catch (const std::invalid_argument& error) {
      throw ConfigError("route." + route.slot + ": " + error.what());
    }
    route.source_body = fields.at("source_body");
    route.canonical_body = fields.at("canonical_body");
    route.sample_period_ns = parseUnsigned(
        "route." + route.slot + ".sample_period_ns", fields.at("sample_period_ns"));
    route.source_uncertainty_ns = parseUnsigned(
        "route." + route.slot + ".source_uncertainty_ns",
        fields.at("source_uncertainty_ns"));
    config.routes.push_back(std::move(route));
  }
  validateRoutes(config.run_mode, config.routes);
  validateModeClockPolicy(config);
  validateSourceTiming(config);
  if (ordered_keys != canonicalKeyOrder(config)) {
    throw ConfigError(
        "config keys are valid but not in canonical Core order or spelling");
  }
  return config;
}

}  // namespace xgc_session_clock_guard
