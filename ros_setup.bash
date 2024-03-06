#!/bin/bash

# If provided, takes the first argument and assigns it to DEFAULT_ROS_VERSION
# Otherwise the default value (digit after the dash) is used.
DEFAULT_ROS_VERSION=${1:-1}

# Always prefer environment variable ROS_VERSION over default value
ROS_VERSION=${ROS_VERSION:-$DEFAULT_ROS_VERSION}

ROS1_WS_DIR="ros/ros1_ws"
ROS2_WS_DIR="ros/ros2_ws"

# echo command before executing it
print_and_execute() { echo "  -> run '${1}'"; eval ${1}; }

if [ ${ROS_VERSION} -eq 1 ] ; then 
    print_and_execute "source \"${ROS1_WS_DIR}/devel/setup.bash\""
elif [ ${ROS_VERSION} -eq 2 ] ; then 
    print_and_execute "source \"${ROS2_WS_DIR}/install/setup.bash\""
else
    echo -e "\033[1;31m  Warning: Wrong ROS Version! \033[0m"
    echo "First argument should either be '1', '2' or empty for default value."
fi

unset DEFAULT_ROS_VERSION
unset -f print_and_execute

echo "  -> LD_LIBRARY_PATH=${LD_LIBRARY_PATH}"
