#!/usr/bin/env bash

set -e

mkdir lib
cd lib
git clone https://github.com/yixuan/LBFGSpp
git clone git@github.com:f1tenth/range_libc.git -b foxy-devel
