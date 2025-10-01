// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2024 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

/**
 * @file main.cpp
 * @brief Main executable for perception node
 * 
 * This file contains modified code originally from apriltag_detector.
 * Modification: Support for our custom node (variation of the original)
 * 
 * 
 * @author Jackson Russell
 * @author Ace Viray
 * @date Oct-2025
 */

#include "perception_node.h"
#include <memory>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<rs1_perception::PerceptionNode>(
      rclcpp::NodeOptions());
      
  RCLCPP_INFO(node->get_logger(), "Starting RS1 Perception Node...");
  
  rclcpp::spin(node);
  
  RCLCPP_INFO(node->get_logger(), "Shutting down RS1 Perception Node");
  rclcpp::shutdown();
  
  return 0;
}
