#!/bin/bash

# Exit immediately if a command exits with a non-zero status.
set -e

# If mainly using one ROS distro, it may come in handy to set a symbolic link
# to your preferred distro:
#  -> sudo ln -sr /opt/ros/<distro> /opt/ros/ros2

# When switching between ROS workspaces using different distros, set ROS_DISTRO
# accordingly before running this script or alter the default value below.

# Always prefer environment variable ROS_DISTRO 
# over default value (comes after the dash).
ROS_DISTRO_NAME=${ROS_DISTRO:-ros2}

# Build command for ROS workspace
BUILD_CMD="colcon build"

# echo command before executing it
print_and_execute() { echo "  -> run '${1}'"; eval ${1}; }

print_and_execute "source \"/opt/ros/${ROS_DISTRO_NAME}/setup.bash\""
print_and_execute "${BUILD_CMD} $@"

unset ROS_DISTRO_NAME BUILD_CMD
unset -f print_and_execute

