#include "xgc_session_clock_guard/epoch_fence.hpp"

#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <cstring>
#include <limits>
#include <string_view>

namespace xgc_session_clock_guard {
namespace {

constexpr const char *kEpochStateSchema =
    "xgc.session-clock-policy.epoch-state.v1";
constexpr std::size_t kMaximumEpochStateBytes = 64U << 10U;
constexpr const char *kEpochStateFileName = "epoch-state.json";
constexpr const char *kEpochLockFileName = "epoch-state.lock";

class FileDescriptor {
public:
  explicit FileDescriptor(int value) : value_(value) {}
  ~FileDescriptor() {
    if (value_ >= 0) {
      ::close(value_);
    }
  }
  FileDescriptor(const FileDescriptor &) = delete;
  FileDescriptor &operator=(const FileDescriptor &) = delete;
  int get() const { return value_; }

private:
  int value_{-1};
};

bool lowerHexDigest(const std::string &value) {
  if (value.size() != 64U) {
    return false;
  }
  for (const unsigned char character : value) {
    if (!((character >= '0' && character <= '9') ||
          (character >= 'a' && character <= 'f'))) {
      return false;
    }
  }
  return true;
}

bool continuation(unsigned char value) {
  return value >= 0x80U && value <= 0xbfU;
}

std::size_t canonicalUtf8Length(const std::string &input,
                                std::size_t position) {
  const auto lead = static_cast<unsigned char>(input[position]);
  if (lead <= 0x7fU) {
    return 1U;
  }
  if (lead >= 0xc2U && lead <= 0xdfU && position + 1U < input.size() &&
      continuation(static_cast<unsigned char>(input[position + 1U]))) {
    return 2U;
  }
  if (lead >= 0xe0U && lead <= 0xefU && position + 2U < input.size()) {
    const auto second = static_cast<unsigned char>(input[position + 1U]);
    const auto third = static_cast<unsigned char>(input[position + 2U]);
    if (continuation(second) && continuation(third) &&
        !(lead == 0xe0U && second < 0xa0U) &&
        !(lead == 0xedU && second >= 0xa0U)) {
      return 3U;
    }
  }
  if (lead >= 0xf0U && lead <= 0xf4U && position + 3U < input.size()) {
    const auto second = static_cast<unsigned char>(input[position + 1U]);
    const auto third = static_cast<unsigned char>(input[position + 2U]);
    const auto fourth = static_cast<unsigned char>(input[position + 3U]);
    if (continuation(second) && continuation(third) && continuation(fourth) &&
        !(lead == 0xf0U && second < 0x90U) &&
        !(lead == 0xf4U && second > 0x8fU)) {
      return 4U;
    }
  }
  return 0U;
}

unsigned int hexValue(unsigned char value) {
  if (value >= '0' && value <= '9') {
    return value - '0';
  }
  if (value >= 'a' && value <= 'f') {
    return 10U + value - 'a';
  }
  return std::numeric_limits<unsigned int>::max();
}

void appendUtf8(std::string *output, unsigned int code_point) {
  if (code_point <= 0x7fU) {
    output->push_back(static_cast<char>(code_point));
  } else if (code_point <= 0x7ffU) {
    output->push_back(static_cast<char>(0xc0U | (code_point >> 6U)));
    output->push_back(static_cast<char>(0x80U | (code_point & 0x3fU)));
  } else {
    output->push_back(static_cast<char>(0xe0U | (code_point >> 12U)));
    output->push_back(static_cast<char>(0x80U | ((code_point >> 6U) & 0x3fU)));
    output->push_back(static_cast<char>(0x80U | (code_point & 0x3fU)));
  }
}

unsigned int utf8CodePoint(const std::string &input, std::size_t position,
                           std::size_t length) {
  const auto first = static_cast<unsigned char>(input[position]);
  if (length == 1U) {
    return first;
  }
  unsigned int result =
      first & (length == 2U ? 0x1fU : length == 3U ? 0x0fU : 0x07U);
  for (std::size_t index = 1U; index < length; ++index) {
    result = (result << 6U) |
             (static_cast<unsigned char>(input[position + index]) & 0x3fU);
  }
  return result;
}

bool goTrimSpaceCodePoint(unsigned int value) {
  // This is Unicode White_Space, the set used by Go strings.TrimSpace.
  return (value >= 0x0009U && value <= 0x000dU) || value == 0x0020U ||
         value == 0x0085U || value == 0x00a0U || value == 0x1680U ||
         (value >= 0x2000U && value <= 0x200aU) || value == 0x2028U ||
         value == 0x2029U || value == 0x202fU || value == 0x205fU ||
         value == 0x3000U;
}

class CanonicalJsonCursor {
public:
  explicit CanonicalJsonCursor(const std::string &bytes) : bytes_(bytes) {}

  void literal(std::string_view expected) {
    if (bytes_.compare(position_, expected.size(), expected) != 0) {
      throw EpochFenceError("epoch-state.json has noncanonical keys or order");
    }
    position_ += expected.size();
  }

  std::string stringValue() {
    literal("\"");
    std::string decoded;
    while (position_ < bytes_.size()) {
      const auto character = static_cast<unsigned char>(bytes_[position_]);
      if (character == '"') {
        ++position_;
        return decoded;
      }
      if (character == '\\') {
        parseEscape(&decoded);
        continue;
      }
      if (character < 0x20U || character == '<' || character == '>' ||
          character == '&') {
        throw EpochFenceError(
            "epoch-state.json string does not use canonical Go JSON escaping");
      }
      const std::size_t length = canonicalUtf8Length(bytes_, position_);
      if (length == 0U) {
        throw EpochFenceError("epoch-state.json contains invalid UTF-8");
      }
      if (length == 3U && character == 0xe2U &&
          static_cast<unsigned char>(bytes_[position_ + 1U]) == 0x80U &&
          (static_cast<unsigned char>(bytes_[position_ + 2U]) == 0xa8U ||
           static_cast<unsigned char>(bytes_[position_ + 2U]) == 0xa9U)) {
        throw EpochFenceError("epoch-state.json must escape U+2028 and U+2029");
      }
      decoded.append(bytes_, position_, length);
      position_ += length;
    }
    throw EpochFenceError("epoch-state.json contains an unterminated string");
  }

  bool finished() const { return position_ == bytes_.size(); }

private:
  void parseEscape(std::string *decoded) {
    ++position_;
    if (position_ >= bytes_.size()) {
      throw EpochFenceError("epoch-state.json ends in an escape");
    }
    const char escape = bytes_[position_++];
    switch (escape) {
    case '"':
      decoded->push_back('"');
      return;
    case '\\':
      decoded->push_back('\\');
      return;
    case 'b':
      decoded->push_back('\b');
      return;
    case 'f':
      decoded->push_back('\f');
      return;
    case 'n':
      decoded->push_back('\n');
      return;
    case 'r':
      decoded->push_back('\r');
      return;
    case 't':
      decoded->push_back('\t');
      return;
    case 'u':
      break;
    default:
      throw EpochFenceError(
          "epoch-state.json contains a noncanonical JSON escape");
    }
    if (position_ + 4U > bytes_.size()) {
      throw EpochFenceError("epoch-state.json has a truncated unicode escape");
    }
    unsigned int code_point = 0U;
    for (std::size_t index = 0U; index < 4U; ++index) {
      const unsigned int digit =
          hexValue(static_cast<unsigned char>(bytes_[position_ + index]));
      if (digit == std::numeric_limits<unsigned int>::max()) {
        throw EpochFenceError(
            "epoch-state.json unicode escapes must use lowercase hex");
      }
      code_point = (code_point << 4U) | digit;
    }
    const std::string spelling = bytes_.substr(position_, 4U);
    position_ += 4U;
    const bool html_escape =
        spelling == "003c" || spelling == "003e" || spelling == "0026";
    const bool separator_escape = spelling == "2028" || spelling == "2029";
    const bool control_escape = code_point < 0x20U && code_point != 0x08U &&
                                code_point != 0x09U && code_point != 0x0aU &&
                                code_point != 0x0cU && code_point != 0x0dU &&
                                spelling[0] == '0' && spelling[1] == '0';
    if (!html_escape && !separator_escape && !control_escape) {
      throw EpochFenceError(
          "epoch-state.json unicode escape is not canonical Go JSON");
    }
    appendUtf8(decoded, code_point);
  }

  const std::string &bytes_;
  std::size_t position_{0U};
};

bool canonicalIdentity(const std::string &value, std::size_t maximum) {
  if (value.empty() || value.size() > maximum ||
      value.find('\0') != std::string::npos) {
    return false;
  }
  std::size_t position = 0U;
  unsigned int first = 0U;
  unsigned int last = 0U;
  while (position < value.size()) {
    const std::size_t length = canonicalUtf8Length(value, position);
    if (length == 0U) {
      return false;
    }
    const unsigned int code_point = utf8CodePoint(value, position, length);
    if (position == 0U) {
      first = code_point;
    }
    last = code_point;
    position += length;
  }
  return !goTrimSpaceCodePoint(first) && !goTrimSpaceCodePoint(last);
}

std::string readSecureEpochState(const std::string &path) {
  struct stat before {};
  if (::lstat(path.c_str(), &before) != 0) {
    throw EpochFenceError("cannot lstat epoch fence: " +
                          std::string(std::strerror(errno)));
  }
  if (!S_ISREG(before.st_mode) || S_ISLNK(before.st_mode)) {
    throw EpochFenceError("epoch fence must be a regular non-symlink file");
  }
  if ((before.st_mode & 07777) != 0600) {
    throw EpochFenceError("epoch fence must have exact mode 0600");
  }
  if (before.st_size <= 0 ||
      static_cast<std::uint64_t>(before.st_size) > kMaximumEpochStateBytes) {
    throw EpochFenceError("epoch fence must be in (0, 64 KiB]");
  }

  FileDescriptor file(::open(path.c_str(), O_RDONLY | O_CLOEXEC | O_NOFOLLOW));
  if (file.get() < 0) {
    throw EpochFenceError("cannot open epoch fence without following links: " +
                          std::string(std::strerror(errno)));
  }
  struct stat opened {};
  if (::fstat(file.get(), &opened) != 0 || !S_ISREG(opened.st_mode) ||
      (opened.st_mode & 07777) != 0600 || opened.st_dev != before.st_dev ||
      opened.st_ino != before.st_ino || opened.st_size != before.st_size) {
    throw EpochFenceError("epoch fence changed during secure open");
  }

  std::string bytes(static_cast<std::size_t>(opened.st_size), '\0');
  std::size_t offset = 0U;
  while (offset < bytes.size()) {
    const ssize_t count =
        ::read(file.get(), &bytes[offset], bytes.size() - offset);
    if (count <= 0) {
      throw EpochFenceError("cannot read complete epoch fence");
    }
    offset += static_cast<std::size_t>(count);
  }
  struct stat current {};
  if (::lstat(path.c_str(), &current) != 0 || current.st_dev != opened.st_dev ||
      current.st_ino != opened.st_ino || current.st_size != opened.st_size ||
      (current.st_mode & 07777) != 0600) {
    throw EpochFenceError("epoch fence was atomically replaced during read");
  }
  return bytes;
}

} // namespace

std::string epochStatePathForPolicy(const std::string &policy_file) {
  if (policy_file.empty() || policy_file.front() != '/') {
    throw EpochFenceError("policy file must be an absolute trusted path");
  }
  const auto separator = policy_file.find_last_of('/');
  if (separator == std::string::npos || separator == 0U ||
      separator + 1U == policy_file.size()) {
    throw EpochFenceError("policy file has no private Session directory");
  }
  return policy_file.substr(0U, separator + 1U) + kEpochStateFileName;
}

std::string epochLockPathForPolicy(const std::string &policy_file) {
  // Reuse the policy path validation and replace only the fixed sibling name.
  const std::string state_path = epochStatePathForPolicy(policy_file);
  return state_path.substr(0U, state_path.size() -
                                   std::strlen(kEpochStateFileName)) +
         kEpochLockFileName;
}

EpochFenceLease::EpochFenceLease(const std::string &policy_file)
    : path_(epochLockPathForPolicy(policy_file)) {
  struct stat before {};
  if (::lstat(path_.c_str(), &before) != 0) {
    throw EpochFenceError("cannot lstat epoch fence lock: " +
                          std::string(std::strerror(errno)));
  }
  if (!S_ISREG(before.st_mode) || S_ISLNK(before.st_mode) ||
      (before.st_mode & 07777) != 0600 || before.st_size != 0) {
    throw EpochFenceError("epoch fence lock must be an empty regular "
                          "non-symlink file with exact mode 0600");
  }

  descriptor_ = ::open(path_.c_str(), O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
  if (descriptor_ < 0) {
    throw EpochFenceError(
        "cannot open epoch fence lock without following links: " +
        std::string(std::strerror(errno)));
  }
  try {
    struct stat opened {};
    if (::fstat(descriptor_, &opened) != 0 || !S_ISREG(opened.st_mode) ||
        (opened.st_mode & 07777) != 0600 || opened.st_dev != before.st_dev ||
        opened.st_ino != before.st_ino || opened.st_size != 0) {
      throw EpochFenceError("epoch fence lock changed during secure open");
    }
    if (::flock(descriptor_, LOCK_SH | LOCK_NB) != 0) {
      throw EpochFenceError("Core epoch allocation currently owns the "
                            "exclusive epoch fence lock");
    }
    validateCurrent();
  } catch (...) {
    ::close(descriptor_);
    descriptor_ = -1;
    throw;
  }
}

EpochFenceLease::~EpochFenceLease() {
  if (descriptor_ >= 0) {
    (void)::flock(descriptor_, LOCK_UN);
    (void)::close(descriptor_);
  }
}

void EpochFenceLease::validateCurrent() const {
  if (descriptor_ < 0) {
    throw EpochFenceError("epoch fence shared lease is not held");
  }
  struct stat opened {};
  struct stat current {};
  if (::fstat(descriptor_, &opened) != 0 || !S_ISREG(opened.st_mode) ||
      (opened.st_mode & 07777) != 0600 || opened.st_size != 0 ||
      ::lstat(path_.c_str(), &current) != 0 || !S_ISREG(current.st_mode) ||
      S_ISLNK(current.st_mode) || (current.st_mode & 07777) != 0600 ||
      current.st_dev != opened.st_dev || current.st_ino != opened.st_ino ||
      current.st_size != 0) {
    throw EpochFenceError(
        "epoch fence lock path no longer names the held regular 0600 inode");
  }
}

EpochFenceState parseCanonicalEpochState(const std::string &bytes) {
  if (bytes.empty() || bytes.size() > kMaximumEpochStateBytes ||
      bytes.back() != '\n') {
    throw EpochFenceError(
        "epoch-state.json must be bounded canonical JSON ending in LF");
  }
  CanonicalJsonCursor cursor(bytes);
  EpochFenceState state;
  cursor.literal("{\"epochId\":");
  const std::string epoch_text = cursor.stringValue();
  cursor.literal(",\"jobId\":");
  state.job_id = cursor.stringValue();
  cursor.literal(",\"policySha256\":");
  state.policy_sha256 = cursor.stringValue();
  cursor.literal(",\"schema\":");
  state.schema = cursor.stringValue();
  cursor.literal(",\"sessionContractSha256\":");
  state.session_contract_sha256 = cursor.stringValue();
  cursor.literal(",\"sessionId\":");
  state.session_id = cursor.stringValue();
  cursor.literal(",\"targetId\":");
  state.target_id = cursor.stringValue();
  cursor.literal("}\n");
  if (!cursor.finished()) {
    throw EpochFenceError("epoch-state.json has trailing data");
  }
  try {
    state.epoch = parseEpochId(epoch_text);
  } catch (const std::exception &error) {
    throw EpochFenceError(std::string("epoch-state.json epochId: ") +
                          error.what());
  }
  if (state.schema != kEpochStateSchema ||
      !canonicalIdentity(state.job_id, 64U) ||
      !canonicalIdentity(state.target_id, 128U) ||
      !lowerHexDigest(state.policy_sha256) ||
      !lowerHexDigest(state.session_contract_sha256) ||
      state.session_id.empty() || state.session_id.size() > 64U) {
    throw EpochFenceError("epoch-state.json identity fields are invalid");
  }
  return state;
}

void validateEpochFenceState(const EpochFenceState &state,
                             const FrozenConfig &config,
                             std::uint64_t expected_epoch) {
  if (state.epoch != expected_epoch ||
      state.policy_sha256 != config.policy_sha256 ||
      state.session_contract_sha256 != config.session_contract_sha256 ||
      state.session_id != config.session_id) {
    throw EpochFenceError(
        "epoch fence no longer matches startup epoch and policy lineage");
  }
}

EpochFenceState loadAndValidateEpochFence(const std::string &policy_file,
                                          const FrozenConfig &config,
                                          std::uint64_t expected_epoch) {
  const EpochFenceState state = parseCanonicalEpochState(
      readSecureEpochState(epochStatePathForPolicy(policy_file)));
  validateEpochFenceState(state, config, expected_epoch);
  return state;
}

} // namespace xgc_session_clock_guard
