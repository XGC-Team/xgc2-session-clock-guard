#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

for command in g++ python3; do
  command -v "$command" >/dev/null 2>&1 || {
    echo "missing C++ quality dependency: $command" >&2
    exit 1
  }
done
FORMATTER="$("$SCRIPT_DIR/require_clang_format_10.sh")"
readonly FORMATTER

mapfile -t cpp_files < <(
  find "$REPO_ROOT/include" "$REPO_ROOT/src" "$REPO_ROOT/test" \
    -type f \( \
      -name '*.c' -o -name '*.cc' -o -name '*.cpp' -o -name '*.cxx' -o \
      -name '*.h' -o -name '*.hh' -o -name '*.hpp' -o -name '*.hxx' \
    \) -print | sort
)
(( ${#cpp_files[@]} > 0 )) || {
  echo "C++ quality gate found no source files" >&2
  exit 1
}
"$FORMATTER" --dry-run --Werror "${cpp_files[@]}"

"$REPO_ROOT/tests/run_core_tests.sh"

echo "C++ quality check passed"
