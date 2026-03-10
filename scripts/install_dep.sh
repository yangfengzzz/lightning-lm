#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS2_DISTRO:-${ROS_DISTRO:-humble}}"

sudo apt install -y \
  libopencv-dev \
  libpcl-dev \
  pcl-tools \
  libyaml-cpp-dev \
  libgoogle-glog-dev \
  libgflags-dev \
  "ros-${ROS_DISTRO}-pcl-conversions"
