// Copyright 2021 ROBOTIS CO., LTD.
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

#ifndef READ_WRITE_NODE_HPP_
#define READ_WRITE_NODE_HPP_

#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "lib_dynamixel.h"

#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/set_id.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/set_speed.hpp"

#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_temperature.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_voltage.hpp"


// extern lib_dynamixel AX12A;

class ReadWriteNode : public rclcpp::Node
{
public:
  using SetPosition = dynamixel_sdk_custom_interfaces::msg::SetPosition;
  using SetId       = dynamixel_sdk_custom_interfaces::msg::SetId;
  using SetSpeed    = dynamixel_sdk_custom_interfaces::msg::SetSpeed;

  using GetPosition = dynamixel_sdk_custom_interfaces::srv::GetPosition;
  using GetTemperature = dynamixel_sdk_custom_interfaces::srv::GetTemperature;
  using GetVoltage = dynamixel_sdk_custom_interfaces::srv::GetVoltage;

  ReadWriteNode();
  virtual ~ReadWriteNode();

private:
  rclcpp::Subscription<SetPosition>::SharedPtr set_position_subscriber_;
  rclcpp::Subscription<SetId>::SharedPtr set_id_subscriber_;
  rclcpp::Subscription<SetSpeed>::SharedPtr set_speed_subscriber_;

  rclcpp::Service<GetPosition>::SharedPtr get_position_server_;
  rclcpp::Service<GetTemperature>::SharedPtr get_temperature_server_;
  rclcpp::Service<GetVoltage>::SharedPtr get_voltage_server_;

  void init_set_position_subscriber(const rclcpp::QoS QOS_RKL10V);
  void init_set_id_subscriber(const rclcpp::QoS QOS_RKL10V);
  void init_set_speed_subscriber(const rclcpp::QoS QOS_RKL10V);

  void init_get_position_server();
  void init_get_temperature_server();
  void init_get_voltage_server();

  int present_position;
};

void init_Dynamixel();
void fin_dynamixel();
#endif  // READ_WRITE_NODE_HPP_
