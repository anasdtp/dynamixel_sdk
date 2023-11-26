#include "read_write_node.h"


// lib_dynamixel AX12A;

int main(int argc, char *argv[])
{
  init_Dynamixel();
  // Setup Dynamixel : ---------------------------------------------------
  // AX12A.begin(BAUDRATE);
  // AX12A.torque(BROADCASTID);
  // AX12A.setSpeed(BROADCASTID, 1000);

  // // sleep(10);
  // AX12A.move(3, 100);
  // sleep(2);
  // AX12A.moveSpeed(3, 1000, 1);

  // Fin setup Dynamixel : ---------------------------------------------------
  rclcpp::init(argc, argv);

  auto readwritenode = std::make_shared<ReadWriteNode>();
  rclcpp::spin(readwritenode);
  rclcpp::shutdown();

  // AX12A.torque(BROADCASTID, false);
  fin_dynamixel();

  return 0;
}