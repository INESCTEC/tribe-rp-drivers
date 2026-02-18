#!/bin/bash

set -e

MODE="format"
if [ "$1" == "--check" ]; then
    MODE="check"
fi

# Finds git repository root and checks if we're inside a git repository
GIT_ROOT=$(git rev-parse --show-toplevel 2>/dev/null)
if [ -z "$GIT_ROOT" ]; then
    echo "Error: You're not in a git repository!"
    exit 1
fi

echo "clang-format mode: $MODE"
echo "Repo root: $GIT_ROOT"

# Find all C/C++ source files, excluding certain directories
FILES=$(find "$GIT_ROOT" \( \
    -name '*.cpp' -o -name '*.h' -o -name '*.c' -o -name '*.hpp' -o -name '*.cc' \
\) \
    -not -path "*/third_party/*" \
    -not -path "*/build/*" \
    -not -path "*/cmake-build-*/*" \
    -not -path "*/.git/*"
)

# Check if any files were found
if [ -z "$FILES" ]; then
    echo "No C/C++ files found."
    exit 0
fi

# Run clang-format on the found files
if [ "$MODE" == "check" ]; then
    echo "Checking formatting..."
    echo "$FILES" | xargs clang-format --dry-run --Werror
else
    echo "Formatting files..."
    echo "$FILES" | xargs clang-format -i
fi

echo "clang-format done ✅"
