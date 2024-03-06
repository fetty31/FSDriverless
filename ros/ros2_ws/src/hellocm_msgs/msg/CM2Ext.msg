#
# Message from CarMaker ROS Node to external ROS Node
#

# General ROS Header (optional)
std_msgs/Header header

# ROS time when message was sent (optional)
builtin_interfaces/Time time

# Cycle number since simulation start (optional)
int64 cycleno

# Synthetic delay to check synchronization (optional)
float64 synthdelay
