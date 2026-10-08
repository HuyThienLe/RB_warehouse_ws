#!/usr/bin/env bash
set -e
sudo apt update
sudo apt install -y \
  ros-humble-desktop ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-robot-localization \
  ros-humble-twist-mux ros-humble-xacro ros-humble-cv-bridge \
  ros-humble-tf2-tools ros-humble-tf2-ros-py ros-humble-joint-state-publisher-gui \
  ros-humble-teleop-twist-keyboard ros-humble-rosbridge-suite \
  python3-colcon-common-extensions python3-rosdep python3-pip python3-opencv python3-scipy
# Ghim numpy<2 để cv_bridge (build với numpy 1.x) không bị lỗi
pip3 install "numpy<2" fastapi "uvicorn[standard]" pandas matplotlib pytest httpx
