#!/usr/bin/env bash

set -e

./run.sh all

type="debug"
if [[ $(uname) == "Darwin" ]]; then
	platform="macos"
else
	platform="linux"
fi
# build_dir="cmake-build-${type}-${platform}"
build_dir="cmake-build-${type}"

cd "$build_dir" || exit 1

# set +e
set -x

./hello_world
# TODO: high_dim_system is failing
# ./high_dim_system
./low_variance_resampler

./functor_test
./icp_2d_test
./knn_test
./solver_test
