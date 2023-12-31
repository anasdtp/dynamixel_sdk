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

/*******************************************************************************
// This example is written for DYNAMIXEL X(excluding XL-320) and MX(2.0) series with U2D2.
// For other series, please refer to the product eManual and modify the Control Table addresses and other definitions.
// To test this example, please follow the commands below.
//
// Open terminal #1
// $ ros2 run dynamixel_sdk_examples read_write_node
//
// Open terminal #2 (run one of below commands at a time)
// $ ros2 topic pub -1 /set_position dynamixel_sdk_custom_interfaces/SetPosition "{id: 2, position: 1000}"
// $ ros2 topic pub -1 /set_id dynamixel_sdk_custom_interfaces/SetId "{id: 254, idnew: 3}"
// $ ros2 topic pub -1 /set_speed dynamixel_sdk_custom_interfaces/SetSpeed "{id: 3, speed: 1000}"
//
// $ ros2 service call /get_position dynamixel_sdk_custom_interfaces/srv/GetPosition "id: 2"
// $ ros2 service call /get_temperature dynamixel_sdk_custom_interfaces/srv/GetTemperature "id: 2"
// $ ros2 service call /get_voltage dynamixel_sdk_custom_interfaces/srv/GetVoltage "id: 2"
//
// Author: Will Son - Anasdtp
*******************************************************************************/

#include <cstdio>
#include <memory>
#include <string>

#include "read_write_node.h"

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




lib_dynamixel AX12A;

ReadWriteNode::ReadWriteNode()
    : Node("read_write_node")
{
  RCLCPP_INFO(this->get_logger(), "Run read write node");

  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = 0;
  this->get_parameter("qos_depth", qos_depth);

  const auto QOS_RKL10V = rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  init_set_position_subscriber(QOS_RKL10V);
  init_set_id_subscriber(QOS_RKL10V);
  init_set_speed_subscriber(QOS_RKL10V);

  init_get_position_server();
  init_get_temperature_server();
  init_get_voltage_server();
}

ReadWriteNode::~ReadWriteNode()
{
}


/// @brief 
/// @param QOS_RKL10V 
void ReadWriteNode::init_set_position_subscriber(const rclcpp::QoS QOS_RKL10V){
  set_position_subscriber_ = this->create_subscription<SetPosition>(
      "set_position",
      QOS_RKL10V,
      [this](const SetPosition::SharedPtr msg) -> void
      {
        // Position Value of X series is 4 byte data.
        // For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
        int goal_position = (unsigned int)msg->position; // Convert int32 -> uint32

        if (AX12A.move((uint8_t)msg->id, goal_position))
        {
          RCLCPP_INFO(this->get_logger(), "Erreur lors du controle du Dynamixel n°%d", msg->id);
        }
        else
        {
          RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Position: %d]", msg->id, msg->position);
        }
      });
}
void ReadWriteNode::init_set_id_subscriber(const rclcpp::QoS QOS_RKL10V){
  set_id_subscriber_ = this->create_subscription<SetId>(
      "set_id",
      QOS_RKL10V,
      [this](const SetId::SharedPtr msg) -> void
      {

        if (AX12A.setID(msg->id, msg->idnew))
        {
          RCLCPP_INFO(this->get_logger(), "Erreur lors du changement de l'id du Dynamixel n°%d", msg->id);
        }
        else
        {
          RCLCPP_INFO(this->get_logger(), "Set id, [New ID: %d]", msg->idnew);
        }
      });
}
void ReadWriteNode::init_set_speed_subscriber(const rclcpp::QoS QOS_RKL10V){
  set_speed_subscriber_ = this->create_subscription<SetSpeed>(
      "set_speed",
      QOS_RKL10V,
      [this](const SetSpeed::SharedPtr msg) -> void
      {
        if (AX12A.setSpeed(msg->id, msg->speed))
        {
          RCLCPP_INFO(this->get_logger(), "Erreur lors du reglage de la vitesse du Dynamixel n°%d", msg->id);
        }
        else
        {
          RCLCPP_INFO(this->get_logger(), "Set Speed [ID: %d] [Speed: %d]", msg->id, msg->speed);
        }
      });
}

void ReadWriteNode::init_get_position_server(){
  auto get_present_position = [this](const std::shared_ptr<GetPosition::Request> request,
                                     std::shared_ptr<GetPosition::Response> response) -> void
  {
    // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
    // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
    present_position = AX12A.readPosition((uint8_t)request->id);

    RCLCPP_INFO(this->get_logger(),
                "Get [ID: %d] [Present Position: %d]",
                request->id,
                present_position);

    response->position = present_position;
  };

  get_position_server_ = create_service<GetPosition>("get_position", get_present_position);
}
void ReadWriteNode::init_get_temperature_server(){
  auto get_present_temperature = [this](const std::shared_ptr<GetTemperature::Request> request,
                                     std::shared_ptr<GetTemperature::Response> response) -> void
  {
    int present_temperature = AX12A.readTemperature(request->id);

    RCLCPP_INFO(this->get_logger(),
                "read Temperature [ID: %d] [Present Temperature: %d]",
                request->id,
                present_temperature);

    response->temperature = present_temperature;
  };

  get_temperature_server_ = create_service<GetTemperature>("get_temperature", get_present_temperature);
}
void ReadWriteNode::init_get_voltage_server(){
  auto get_present_voltage = [this](const std::shared_ptr<GetVoltage::Request> request,
                                     std::shared_ptr<GetVoltage::Response> response) -> void
  {
    int present_voltage = AX12A.readVoltage((uint8_t)request->id);

    RCLCPP_INFO(this->get_logger(),
                "read Voltage [ID: %d] [Present Voltage: %d]",
                request->id,
                present_voltage);

    response->voltage = present_voltage;
  };

  get_voltage_server_ = create_service<GetVoltage>("get_voltage", get_present_voltage);
}


void init_Dynamixel(){
  // Setup Dynamixel : ---------------------------------------------------
  AX12A.begin(BAUDRATE);
  AX12A.torque(BROADCASTID);
  AX12A.setSpeed(BROADCASTID, 1000);

  // // sleep(10);
  // AX12A.move(3, 100);
  // sleep(2);
  // AX12A.moveSpeed(3, 1000, 1);

  // Fin setup Dynamixel : ---------------------------------------------------
}

void fin_dynamixel(){
  AX12A.torque(BROADCASTID, false);
}
