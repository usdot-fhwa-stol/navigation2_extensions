# `carma_nav2_port_drayage_demo` package

This package implements port drayage integration with a Nav2-based autonomy stack. The package's node listens for
incoming port drayage variants of `MobilityOperation.msg` messages then sends the target location to the Nav2 Waypoint
Follower via a ROS 2 action call.
