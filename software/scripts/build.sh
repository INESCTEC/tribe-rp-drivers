#!/bin/bash

set -e # abort at the first error

# Directory where the script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
echo "Script dir: $SCRIPT_DIR"

# Compilation directory
BUILD_DIR="${SCRIPT_DIR}/../build"
echo "Build dir:  $BUILD_DIR"

# Create a compilation directory if it doesn't exist
mkdir -p "${BUILD_DIR}"

# Navigate to the compilation directory
cd "${BUILD_DIR}"

# Configure the project
cmake .. -DCMAKE_BUILD_TYPE=Release

# Compile and install third-party
for dir in third_party interfaces drivers examples; do
  echo "==> Building $dir"
  cd "$BUILD_DIR/$dir"
  make -j$(nproc)
  make install
done
