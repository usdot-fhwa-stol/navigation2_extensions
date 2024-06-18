# `carma_nav2_port_drayage_demo` package

This package implements port drayage integration with a Nav2-based autonomy stack. The package's node listens for
incoming port drayage variants of `MobilityOperation.msg` messages then sends the target location to the Nav2 Waypoint
Follower via a ROS 2 action call.

## Turtlebot Gazebo Launch Instructions

To run the Port Drayage demo using the Turtlebot in the Gazebo simulator, first follow [this tutorial](https://roboticsbackend.com/ros2-nav2-tutorial/)
to correctly configure, launch, and map the simulator. Once you have created a map of the simulation environment, you can start the Port Drayage demo with

```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=path/to/my_map.yaml
ros2 run carma_nav2_port_drayage_demo carma_nav2_port_drayage_demo_node
ros2 lifecycle set /carma_nav2_port_drayage_demo_node configure
ros2 lifecycle set /carma_nav2_port_drayage_demo_node activate
```

Next, provide the Turtlebot with an initial pose estimate using the 2D Pose Estimate arrow in Rviz. Once the vehicle has localized, it is now ready to
receive a Mobility Operation message to determine a goal to reach. To publish this manually, use

```
ros2 topic pub --once /incoming_mobility_operation carma_v2x_msgs/msg/MobilityOperation "m_header:
  sender_id: ''
  recipient_id: ''
  sender_bsm_id: ''
  plan_id: ''
  timestamp: 0
strategy: 'carma/port_drayage'
strategy_params: '{\"destination\":{\"longitude\":X_COORD_DEST,\"latitude\":Y_COORD_DEST}}'"
```

with `X_COORD_DEST` and `Y_COORD_DEST` replaced with `x` and `y` coordinates of the desired goal. The Turtlebot should then begin driving to the specified
destination and publish a Mobility Operation message acknowledging that it reached the goal on `/outgoing_mobility_operation`.
