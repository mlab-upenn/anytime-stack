#!/usr/bin/env bash

set -e

cd libs
colcon build --merge-install --packages-up-to Eigen3 lbfgspp
# colcon build --merge-install --packages-up-to Eigen3 lbfgspp range_lib --no-warn-unused-cli --cmake-args -DWITH_CUDA=ON
# colcon build --merge-install --packages-up-to Eigen3 lbfgspp range_lib --no-warn-unused-cli --cmake-args -DWITH_CUDA=OFF
