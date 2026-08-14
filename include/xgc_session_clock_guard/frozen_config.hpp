#pragma once

#include <stdexcept>
#include <string>

#include "xgc_session_clock_guard/types.hpp"

namespace xgc_session_clock_guard {

class ConfigError : public std::runtime_error {
public:
  explicit ConfigError(const std::string &message)
      : std::runtime_error(message) {}
};

std::string sha256Hex(const std::string &bytes);

class FrozenConfigLoader {
public:
  static FrozenConfig loadFile(const std::string &path,
                               const std::string &expected_sha256);
  static FrozenConfig parse(const std::string &bytes,
                            const std::string &verified_sha256);
};

void validateThresholdPolicy(const ThresholdPolicy &policy);
void validateRoutes(const std::string &run_mode,
                    const std::vector<Route> &routes);
void validateModeClockPolicy(const FrozenConfig &config);
void validateSourceTiming(const FrozenConfig &config);

} // namespace xgc_session_clock_guard
