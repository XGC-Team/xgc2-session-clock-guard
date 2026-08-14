#pragma once

#include <cstdint>
#include <stdexcept>
#include <string>

#include "xgc_session_clock_guard/types.hpp"

namespace xgc_session_clock_guard {

class EpochFenceError : public std::runtime_error {
public:
  explicit EpochFenceError(const std::string &message)
      : std::runtime_error(message) {}
};

struct EpochFenceState {
  std::uint64_t epoch{0};
  std::string job_id;
  std::string policy_sha256;
  std::string schema;
  std::string session_contract_sha256;
  std::string session_id;
  std::string target_id;
};

std::string epochStatePathForPolicy(const std::string &policy_file);
std::string epochLockPathForPolicy(const std::string &policy_file);

// Holds the shared side of the Core/Guard epoch-allocation fence for the
// complete process lifetime. Core must obtain LOCK_EX|LOCK_NB on the same
// inode before it may create or replace epoch-state.json.
class EpochFenceLease {
public:
  explicit EpochFenceLease(const std::string &policy_file);
  ~EpochFenceLease();

  EpochFenceLease(const EpochFenceLease &) = delete;
  EpochFenceLease &operator=(const EpochFenceLease &) = delete;
  EpochFenceLease(EpochFenceLease &&) = delete;
  EpochFenceLease &operator=(EpochFenceLease &&) = delete;

  void validateCurrent() const;
  const std::string &path() const { return path_; }

private:
  std::string path_;
  int descriptor_{-1};
};

EpochFenceState parseCanonicalEpochState(const std::string &bytes);
void validateEpochFenceState(const EpochFenceState &state,
                             const FrozenConfig &config,
                             std::uint64_t expected_epoch);
EpochFenceState loadAndValidateEpochFence(const std::string &policy_file,
                                          const FrozenConfig &config,
                                          std::uint64_t expected_epoch);

} // namespace xgc_session_clock_guard
