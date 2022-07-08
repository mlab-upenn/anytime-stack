#!/usr/bin/env bash

set -e

rm -rf libs
mkdir -p libs/src
cd libs/src
git clone https://github.com/yixuan/LBFGSpp.git -b master lbfgspp
git clone git@github.com:f1tenth/range_libc.git -b foxy-devel range_lib
git clone https://gitlab.com/libeigen/eigen.git -b 3.4.0 Eigen3
