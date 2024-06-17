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

namespace carma_nav2_port_drayage_demo
{

// std::string PortDrayageDemo::Operation::operationToString() const {

//   // Convert operation enum into a human-readable string
//   switch(operation_enum_) {
//     case Operation::PICKUP:             return "PICKUP";
//     case Operation::DROPOFF:            return "DROPOFF";
//     case Operation::ENTER_STAGING_AREA: return "ENTER_STAGING_AREA";
//     case Operation::EXIT_STAGING_AREA:  return "EXIT_STAGING_AREA";
//     case Operation::ENTER_PORT:         return "ENTER_PORT";
//     case Operation::EXIT_PORT:          return "EXIT_PORT";
//     case Operation::PORT_CHECKPOINT:    return "PORT_CHECKPOINT";
//     case Operation::HOLDING_AREA:       return "HOLDING_AREA";
//     default:
//       RCLCPP_WARN_STREAM(rclcpp::get_logger("OperationID"), "Conversion of an unsupported operation enum value to a string.");
//       return "UNSUPPORTED_OPERATION_ID";
//   }
// }

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
    "incoming_mobility_operation", 1, [this](const carma_v2x_msgs::msg::MobilityOperation & msg) {
      on_mobility_operation_received(msg);
    });

  odometry_subscription_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "amcl_pose", 1, [this](const geometry_msgs::msg::PoseWithCovarianceStamped & msg) {
      on_odometry_received(msg);
    });

  mobility_operation_publisher_ = create_publisher<carma_v2x_msgs::msg::MobilityOperation>(
    "outgoing_mobility_operation", 1);

  // current_operation_ = PortDrayageDemo::Operation::ENTER_STAGING_AREA;
  // nlohmann::json mobility_operation_json, location_json;
  // mobility_operation_json["cmv_id"] = "C1T-Truck";
  // mobility_operation_json["operation"] = current_operation_;
  // mobility_operation_json["cargo"] = false;
  // mobility_operation_json["cargo_id"] = "";
  // location_json["longitude"] = 0.0; // Could set to the initial destination coordinates instead
  // location_json["latitude"] = 0.0;
  // mobility_operation_json["destination"] = location_json;
  // current_strategy_params_ = mobility_operation_json.dump();
  actively_executing_operation_ = false;

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
    RCLCPP_WARN_STREAM(
      get_logger(),
      "Received MobilityOperation with unsupported strategy '"
        << msg.strategy << "'");

    return;
  }

  if (actively_executing_operation_) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "Ignoring received port drayage operation '"
        << msg.strategy_params << "' while already executing the operation '"
        << current_strategy_params_ << "'");

    return;
  }

  try {
    const auto strategy_params_json = nlohmann::json::parse(msg.strategy_params);

    const auto longitude{strategy_params_json["destination"]["longitude"].template get<float>()};
    const auto latitude{strategy_params_json["destination"]["latitude"].template get<float>()};
    const auto operation{strategy_params_json["operation"].template get<std::string>()};

    RCLCPP_INFO_STREAM(get_logger(), longitude << ", " << latitude);

    nav2_msgs::action::FollowWaypoints::Goal goal;

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = longitude;
    pose.pose.position.y = latitude;
    current_operation_ = operation;
    current_strategy_params_ = msg.strategy_params;

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
    {
      RCLCPP_INFO(get_logger(), "Goal successfully reached");
      nlohmann::json mobility_operation_json, location_json;
      mobility_operation_json["cmv_id"] = "C1T-Truck";
      mobility_operation_json["operation"] = current_operation_;
      mobility_operation_json["cargo"] = false;
      mobility_operation_json["cargo_id"] = "";
      location_json["longitude"] = current_odometry_.pose.pose.position.x;
      location_json["latitude"] = current_odometry_.pose.pose.position.y;
      mobility_operation_json["location"] = location_json;
      try {
        const auto current_strategy_params_json = nlohmann::json::parse(current_strategy_params_);
        const auto action_id{current_strategy_params_json["action_id"].template get<std::string>()};
        mobility_operation_json["action_id"] = action_id;
      } catch (const nlohmann::json::exception & e) {
        RCLCPP_ERROR_STREAM(
          get_logger(), "Could not process strategy params while generating port drayage ACK message: JSON error: " << e.what());
      }
      carma_v2x_msgs::msg::MobilityOperation result;
      result.strategy_params = mobility_operation_json.dump();
      mobility_operation_publisher_->publish(std::move(result));
      return;
    }
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

auto PortDrayageDemo::on_odometry_received(const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
  -> void
{
  current_odometry_ = msg;
}

}  // namespace carma_nav2_port_drayage_demo
