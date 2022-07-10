#!/usr/bin/env bash

# CLion handles configuration automatically (via profiles),
# see Preferences / Settings > Build, Execution, Deployment > CMake.
# This script is useful when building outside CLion.

set -e

type="debug"
generator="Ninja"
# generator="Unix Makefiles"

if [[ $(uname) == "Darwin" ]]; then
	platform="macos"
else
	platform="linux"
fi

# build_dir="cmake-build-${type}-${platform}"
build_dir="cmake-build-${type}"

rm -rf "$build_dir"
# ^ > uppercase the first letter, i.e. debug > Debug
cmake -DCMAKE_BUILD_TYPE="${type^}" -G "$generator" -B "$build_dir" "$@" .
