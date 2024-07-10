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
#include <thread>

#include <carma_nav2_port_drayage_demo/port_drayage_demo.hpp>

TEST(PortDrayageTest, dummyTest) { ASSERT_EQ(4, 2 + 2); }

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