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

#include "carma_nav2_port_drayage_demo/port_drayage_demo.hpp"

#include <nav2_msgs/action/follow_waypoints.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <nlohmann/json.hpp>

namespace carma_nav2_port_drayage_demo
{
PortDrayageDemo::PortDrayageDemo(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("port_drayage_demo", options)
{
  RCLCPP_INFO(get_logger(), "Creating");
}

auto PortDrayageDemo::on_configure(const rclcpp_lifecycle::State & /* state */)
  -> nav2_util::CallbackReturn
{
  RCLCPP_INFO(get_logger(), "Configuring");

  follow_waypoints_client_ = rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(
    get_node_base_interface(), get_node_graph_interface(), get_node_logging_interface(),
    get_node_waitables_interface(), "follow_waypoints");

  mobility_operation_subscription_ = create_subscription<carma_v2x_msgs::msg::MobilityOperation>(
    "input/mobility_operation", 1, [this](const carma_v2x_msgs::msg::MobilityOperation & msg) {
      on_mobility_operation_received(msg);
    });

  mobility_operation_publisher_ = create_publisher<carma_v2x_msgs::msg::MobilityOperation>(
    "output/mobility_operation", 1);

  return nav2_util::CallbackReturn::SUCCESS;
}

auto PortDrayageDemo::on_activate(const rclcpp_lifecycle::State & /* state */)
  -> nav2_util::CallbackReturn
{
  RCLCPP_INFO(get_logger(), "Activating");

  return nav2_util::CallbackReturn::SUCCESS;
}

auto PortDrayageDemo::on_deactivate(const rclcpp_lifecycle::State & /* state */)
  -> nav2_util::CallbackReturn
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  return nav2_util::CallbackReturn::SUCCESS;
}

auto PortDrayageDemo::on_mobility_operation_received(
  const carma_v2x_msgs::msg::MobilityOperation & msg) -> void
{
  if (msg.strategy != "carma/port_drayage") {
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "Could not process MobilityOperation message: expected strategy 'carma/port_drayage': "
      "received strategy '"
        << msg.strategy << "'");

    return;
  }

  try {
    const auto strategy_params_json = nlohmann::json::parse(msg.strategy_params);

    const auto longitude{strategy_params_json["destination"]["longitude"].template get<float>()};
    const auto latitude{strategy_params_json["destination"]["latitude"].template get<float>()};

    RCLCPP_INFO_STREAM(get_logger(), longitude << ", " << latitude);

    nav2_msgs::action::FollowWaypoints::Goal goal;

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = longitude;
    pose.pose.position.y = latitude;

    goal.poses.push_back(std::move(pose));

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SendGoalOptions();
    send_goal_options.result_callback =
      std::bind(&PortDrayageDemo::on_result_received, this, std::placeholders::_1);
    follow_waypoints_client_->async_send_goal(goal, send_goal_options);
  } catch (const nlohmann::json::exception & e) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "Could not process MobilityOperation message: JSON error: " << e.what());
  }
}

auto PortDrayageDemo::on_result_received(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::WrappedResult & result) -> void
{
  switch (result.code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(get_logger(), "Goal successfully reached");
      return;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(get_logger(), "Goal aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(get_logger(), "Goal canceled");
      return;
    default:
      RCLCPP_ERROR(get_logger(), "An unknown error occurred");
      return;
  }
}

}  // namespace carma_nav2_port_drayage_demo
