#!/bin/bash
set -e

# Parse command line arguments
QUIET_FLAG=""
if [ "$1" == "--quiet" ]; then
    QUIET_FLAG="--quiet"
fi

# Finds git repository root
GIT_ROOT=$(git rev-parse --show-toplevel 2>/dev/null)

if [ -z "$GIT_ROOT" ]; then
    echo "Error: You're not in a git repository!"
    exit 1
fi

SOFTWARE_DIR="$GIT_ROOT/software"

echo "Running static analysis (cppcheck)..."
echo "Target: $SOFTWARE_DIR"

cppcheck \
  --enable=warning,performance \
  --std=c++17 \
  --error-exitcode=1 \
  --suppress=missingIncludeSystem \
  --inline-suppr \
  $QUIET_FLAG \
  "$SOFTWARE_DIR/drivers" \
  "$SOFTWARE_DIR/interfaces" \
  "$SOFTWARE_DIR/examples"

echo "Static analysis completed successfully ✅"
