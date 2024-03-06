#!/bin/bash

# If provided, takes the nth argument and assigns it to the variable
# Otherwise the default value is used:
# <VARIABLE NAME>=${<#ARGUMENT>:-<DEFAULT VALUE>}
ROS_VERSION=${1:-1}
SRC_DIR=${2:-src}

# echo command before executing it
print_and_execute() { echo -e "Executing: '${1}'\n"; eval ${1}; }

if [ ${ROS_VERSION} -eq 1 ] ; then
    print_and_execute "make -C ${SRC_DIR} cm-ros1 --no-print-directory"
elif [ ${ROS_VERSION} -eq 2 ] ; then
    print_and_execute "make -C ${SRC_DIR} cm-ros2 --no-print-directory"
else
    echo -e "\033[1;31m  Warning: Wrong ROS Version! \033[0m"
    echo "First argument should either be '1', '2' or empty for default value."
fi

unset ROS_VERSION
unset -f print_and_execute
