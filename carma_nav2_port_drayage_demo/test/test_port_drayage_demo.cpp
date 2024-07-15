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

#include <gtest/gtest.h>
#include <chrono>
#include <future>
#include <iostream>
#include <memory>
#include <nlohmann/json.hpp>
#include <thread>

#include <carma_nav2_port_drayage_demo/port_drayage_demo.hpp>

// Test vehicle only responds to incoming mobility operation messages that match its cmv_id
TEST(PortDrayageTest, cmvIdTest)
{
  auto node{std::make_shared<carma_nav2_port_drayage_demo::PortDrayageDemo>(rclcpp::NodeOptions{})};
  node->on_configure(node->get_current_state());
  node->set_cmv_id("test_cmv_id");
  // Create Mobility Operation Message
  carma_v2x_msgs::msg::MobilityOperation cmd;
  cmd.m_header.sender_id = "not_test_cmv_id";
  cmd.strategy = "carma/port_drayage";
  // Set JSON fields
  nlohmann::json mobility_operation_json, location_json;
  location_json["longitude"] = 0.0;
  location_json["latitude"] = 0.0;
  mobility_operation_json["destination"] = location_json;
  mobility_operation_json["cmv_id"] = "not_test_cmv_id";
  mobility_operation_json["operation"] = "PICKUP";
  mobility_operation_json["action_id"] = "some_action";
  mobility_operation_json["cargo_id"] = "new_cargo";
  mobility_operation_json["cargo"] = true;
  cmd.strategy_params = mobility_operation_json.dump();
  node->on_mobility_operation_received(cmd);
  ASSERT_FALSE(node->is_actively_executing_operation());
  cmd.m_header.sender_id = "test_cmv_id";
  mobility_operation_json["cmv_id"] = "test_cmv_id";
  cmd.strategy_params = mobility_operation_json.dump();
  node->on_mobility_operation_received(cmd);
  ASSERT_TRUE(node->is_actively_executing_operation());
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::WrappedResult result;
  result.code = rclcpp_action::ResultCode::SUCCEEDED;
  node->on_result_received(result);
  ASSERT_EQ(node->get_cargo_id(), "new_cargo");
  ASSERT_FALSE(node->is_actively_executing_operation());
}

// Test vehicle only responds to incoming mobility operation messages with strategy carma/port_drayage
TEST(PortDrayageTest, strategyTest)
{
  auto node{std::make_shared<carma_nav2_port_drayage_demo::PortDrayageDemo>(rclcpp::NodeOptions{})};
  node->on_configure(node->get_current_state());
  node->set_cmv_id("test_cmv_id");
  // Create Mobility Operation Message
  carma_v2x_msgs::msg::MobilityOperation cmd;
  cmd.m_header.sender_id = "test_cmv_id";
  cmd.strategy = "some_other_strategy";
  // Set JSON fields
  nlohmann::json mobility_operation_json, location_json;
  location_json["longitude"] = 0.0;
  location_json["latitude"] = 0.0;
  mobility_operation_json["destination"] = location_json;
  mobility_operation_json["cmv_id"] = "test_cmv_id";
  mobility_operation_json["operation"] = "PICKUP";
  mobility_operation_json["action_id"] = "some_action";
  mobility_operation_json["cargo_id"] = "new_cargo";
  mobility_operation_json["cargo"] = true;
  cmd.strategy_params = mobility_operation_json.dump();
  node->on_mobility_operation_received(cmd);
  ASSERT_FALSE(node->is_actively_executing_operation());
  cmd.strategy = "carma/port_drayage";
  node->on_mobility_operation_received(cmd);
  ASSERT_TRUE(node->is_actively_executing_operation());
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::WrappedResult result;
  result.code = rclcpp_action::ResultCode::SUCCEEDED;
  node->on_result_received(result);
  ASSERT_EQ(node->get_cargo_id(), "new_cargo");
  ASSERT_FALSE(node->is_actively_executing_operation());
}

// Test vehicle pickup and dropoff
TEST(PortDrayageTest, pickupAndDropoffTest)
{
  // PICKUP
  auto node{std::make_shared<carma_nav2_port_drayage_demo::PortDrayageDemo>(rclcpp::NodeOptions{})};
  node->on_configure(node->get_current_state());
  node->set_cmv_id("test_cmv_id");
  // Create Mobility Operation Message
  carma_v2x_msgs::msg::MobilityOperation cmd;
  cmd.m_header.sender_id = "test_cmv_id";
  cmd.strategy = "carma/port_drayage";
  // Set JSON fields
  nlohmann::json mobility_operation_json, location_json;
  location_json["longitude"] = 0.0;
  location_json["latitude"] = 0.0;
  mobility_operation_json["destination"] = location_json;
  mobility_operation_json["cmv_id"] = "test_cmv_id";
  mobility_operation_json["operation"] = "PICKUP";
  mobility_operation_json["action_id"] = "some_action";
  mobility_operation_json["cargo_id"] = "new_cargo";
  mobility_operation_json["cargo"] = true;
  cmd.strategy_params = mobility_operation_json.dump();
  node->on_mobility_operation_received(cmd);
  ASSERT_TRUE(node->is_actively_executing_operation());
  ASSERT_EQ(node->get_cargo_id(), "");
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::WrappedResult result;
  result.code = rclcpp_action::ResultCode::SUCCEEDED;
  node->on_result_received(result);
  ASSERT_EQ(node->get_cargo_id(), "new_cargo");
  ASSERT_FALSE(node->is_actively_executing_operation());
  // DROPOFF
  mobility_operation_json["operation"] = "DROPOFF";
  cmd.strategy_params = mobility_operation_json.dump();
  node->on_mobility_operation_received(cmd);
  ASSERT_TRUE(node->is_actively_executing_operation());
  ASSERT_EQ(node->get_cargo_id(), "new_cargo");
  node->on_result_received(result);
  ASSERT_EQ(node->get_cargo_id(), "");
  ASSERT_FALSE(node->is_actively_executing_operation());
}

// Test vehicle ack formatting
TEST(PortDrayageTest, acknowledgementTest)
{
  auto node{std::make_shared<carma_nav2_port_drayage_demo::PortDrayageDemo>(rclcpp::NodeOptions{})};
  node->on_configure(node->get_current_state());
  node->set_cmv_id("test_cmv_id");
  // Create Mobility Operation Message
  carma_v2x_msgs::msg::MobilityOperation cmd;
  cmd.m_header.sender_id = "test_cmv_id";
  cmd.strategy = "carma/port_drayage";
  // Set JSON fields
  nlohmann::json mobility_operation_json, location_json;
  location_json["longitude"] = 0.0;
  location_json["latitude"] = 0.0;
  mobility_operation_json["destination"] = location_json;
  mobility_operation_json["cmv_id"] = "test_cmv_id";
  mobility_operation_json["operation"] = "PICKUP";
  mobility_operation_json["action_id"] = "some_action";
  mobility_operation_json["cargo_id"] = "new_cargo";
  mobility_operation_json["cargo"] = true;
  cmd.strategy_params = mobility_operation_json.dump();
  node->on_mobility_operation_received(cmd);
  carma_v2x_msgs::msg::MobilityOperation ack = node->compose_arrival_message();
  const auto strategy_params_json = nlohmann::json::parse(ack.strategy_params);
  ASSERT_NEAR(node->get_clock()->now().nanoseconds() / 1E6, ack.m_header.timestamp, 1E3);
  ASSERT_EQ(ack.strategy, "carma/port_drayage");
  ASSERT_EQ(strategy_params_json["location"]["longitude"], 0.0);
  ASSERT_EQ(strategy_params_json["location"]["latitude"], 0.0);
  ASSERT_EQ(strategy_params_json["cmv_id"], "test_cmv_id");
  ASSERT_EQ(strategy_params_json["operation"], "PICKUP");
  ASSERT_EQ(strategy_params_json["action_id"], "some_action");
  ASSERT_EQ(strategy_params_json["cargo_id"], "new_cargo");
  ASSERT_TRUE(strategy_params_json["cargo"]);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // Initialize ROS
  rclcpp::init(argc, argv);

  bool success = RUN_ALL_TESTS();

  // Shutdown ROS
  rclcpp::shutdown();

  return success;
}
