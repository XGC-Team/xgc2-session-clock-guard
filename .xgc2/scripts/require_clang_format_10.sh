#!/usr/bin/env bash
set -euo pipefail

readonly FORMATTER=clang-format-10

command -v "$FORMATTER" >/dev/null 2>&1 || {
  echo "required formatter is unavailable: $FORMATTER" >&2
  exit 1
}

version="$($FORMATTER --version)"
[[ "$version" =~ clang-format[[:space:]]version[[:space:]]10[.] ]] || {
  echo "required formatter major is 10, got: $version" >&2
  exit 1
}

command -v "$FORMATTER"
