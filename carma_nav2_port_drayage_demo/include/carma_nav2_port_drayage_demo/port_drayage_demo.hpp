// Copyright 2024 Leidos
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CARMA_NAV2_PORT_DRAYAGE_DEMO__PORT_DRAYAGE_DEMO_HPP_
#define CARMA_NAV2_PORT_DRAYAGE_DEMO__PORT_DRAYAGE_DEMO_HPP_

#include <carma_v2x_msgs/msg/mobility_operation.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <nlohmann/json.hpp>

namespace carma_nav2_port_drayage_demo
{
class PortDrayageDemo : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit PortDrayageDemo(const rclcpp::NodeOptions & options);

  auto on_configure(const rclcpp_lifecycle::State & state) -> nav2_util::CallbackReturn override;

  auto on_activate(const rclcpp_lifecycle::State & state) -> nav2_util::CallbackReturn override;

  auto on_deactivate(const rclcpp_lifecycle::State & state) -> nav2_util::CallbackReturn override;

  auto on_mobility_operation_received(const carma_v2x_msgs::msg::MobilityOperation & msg) -> void;

  auto on_result_received(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::WrappedResult & result) -> void;

  auto on_odometry_received(const geometry_msgs::msg::PoseWithCovarianceStamped & msg) -> void;

  enum Operation {
    PICKUP,
    DROPOFF,
    ENTER_STAGING_AREA,
    EXIT_STAGING_AREA,
    ENTER_PORT,
    EXIT_PORT,
    PORT_CHECKPOINT,
    HOLDING_AREA,
    DEFAULT_OPERATION
  };

private:
  rclcpp::Subscription<carma_v2x_msgs::msg::MobilityOperation>::SharedPtr
    mobility_operation_subscription_{nullptr};

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    odometry_subscription_{nullptr};

  rclcpp::Publisher<carma_v2x_msgs::msg::MobilityOperation>::SharedPtr
    mobility_operation_publisher_{nullptr};

  rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr follow_waypoints_client_{
    nullptr};

  geometry_msgs::msg::PoseWithCovarianceStamped current_odometry_;

  std::string current_operation_;
  std::string current_strategy_params_;
  bool actively_executing_operation_ = false;
};
}  // namespace carma_nav2_port_drayage_demo

#endif  // CARMA_NAV2_PORT_DRAYAGE_DEMO__PORT_DRAYAGE_DEMO_HPP_
