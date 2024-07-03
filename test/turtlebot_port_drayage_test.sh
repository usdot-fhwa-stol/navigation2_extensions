#!/usr/bin/env bash

ros2 topic pub --once /incoming_mobility_operation carma_v2x_msgs/msg/MobilityOperation "m_header:
  sender_id: ''
  recipient_id: ''
  sender_bsm_id: ''
  plan_id: ''
  timestamp: 0
strategy: 'carma/port_drayage'
strategy_params: '{\"cmv_id\":\"turtlebot\",\"operation\":\"PICKUP\",\"cargo\":false,\"cargo_id\":\"C1T_CARGO\",\"destination\":{\"longitude\":3.8,\"latitude\":0.5},\"action_id\":\"PORT_DRAYAGE\"}'"

sleep 50

ros2 topic pub --once /incoming_mobility_operation carma_v2x_msgs/msg/MobilityOperation "m_header:
  sender_id: ''
  recipient_id: ''
  sender_bsm_id: ''
  plan_id: ''
  timestamp: 0
strategy: 'carma/port_drayage'
strategy_params: '{\"cmv_id\":\"turtlebot\",\"operation\":\"DROPOFF\",\"cargo\":false,\"cargo_id\":\"C1T_CARGO\",\"destination\":{\"longitude\":0.0,\"latitude\":1.3},\"action_id\":\"PORT_DRAYAGE\"}'"


