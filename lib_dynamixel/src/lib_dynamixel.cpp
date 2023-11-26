
#include <cstdio>
#include <memory>
#include <string>

#include "dynamixel_sdk/dynamixel_sdk.h"

#include "lib_dynamixel.h"


lib_dynamixel::lib_dynamixel(){
    portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
}

int lib_dynamixel::begin(const int baudrate){
    // Open Serial Port
  dxl_comm_result = portHandler->openPort();
  if (dxl_comm_result == false)
  {
    return -1;
  }


  // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
  dxl_comm_result = portHandler->setBaudRate(baudrate);
  if (dxl_comm_result == false)
  {
    return -1;
  }
  else
  {
    return 0;
  }
}

int lib_dynamixel::reset(uint8_t id){
  dxl_error = 0;

  dxl_comm_result = packetHandler->factoryReset(
    portHandler,
    id, 0,
    &dxl_error);
  
  if (dxl_comm_result != COMM_SUCCESS)
  {
    return -1;
  }
  else if (dxl_error != 0)
  {
    return -1;
  }
  else
  {
    return 0;
  }
}

int lib_dynamixel::setID(uint8_t id, uint8_t newID){
  dxl_error = 0;

  dxl_comm_result = packetHandler->write1ByteTxRx(
          portHandler,
          id,
          AX_ID,
          newID,
          &dxl_error
          );

  if (dxl_comm_result != COMM_SUCCESS)
  {
    return -1;
  }
  else if (dxl_error != 0)
  {
    return -1;
  }
  else
  {
    return 0;
  }
}

void lib_dynamixel::torque(uint8_t id, bool on){

    // Disable or not Torque of DYNAMIXEL
    packetHandler->write1ByteTxRx(
      portHandler,
      id,
      ADDR_TORQUE_ENABLE,
      on,
      &dxl_error);
}

int lib_dynamixel::move(uint8_t id, uint16_t position)
{
  dxl_error = 0;
  // Write Goal Position (length : 4 bytes)
  // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
  dxl_comm_result = packetHandler->write2ByteTxRx(
          portHandler,
          id,
          AX_GOAL_POSITION_L,
          position,
          &dxl_error
          );

  if (dxl_comm_result != COMM_SUCCESS)
  {
    return -1;
  }
  else if (dxl_error != 0)
  {
    return -1;
  }
  else
  {
    return 0;
  }
}

int lib_dynamixel::moveSpeed(uint8_t id, uint16_t position, uint16_t Speed)
{
  dxl_error = 0;

  dxl_comm_result = packetHandler->write4ByteTxRx(
          portHandler,
          id,
          AX_GOAL_POSITION_L,
          (Speed<<16) | position,//4 octets, soit deux variables en un
          &dxl_error
          );

  if (dxl_comm_result != COMM_SUCCESS)
  {
    return -1;
  }
  else if (dxl_error != 0)
  {
    return -1;
  }
  else
  {
    return 0;
  }
}

/*Devrait faire tourner en continu les servos mais utilie AX_GOAL_SPEED_L qui regle juste la vitesse... bon*/
int lib_dynamixel::turn(uint8_t id, bool SIDE, uint16_t Speed)
{
  dxl_error = 0;

  if (SIDE == RIGHT)
  {
    char Speed_H, Speed_L;
    Speed_H = (Speed >> 8) + 4;
    Speed_L = Speed; // 16 bits - 2 x 8 bits variables

    dxl_comm_result = packetHandler->write2ByteTxRx(
        portHandler,
        id,
        AX_GOAL_SPEED_L,
        (Speed_H<<8)|Speed_L,
        &dxl_error);
  }
  else
  {
    dxl_comm_result = packetHandler->write2ByteTxRx(
        portHandler,
        id,
        AX_GOAL_SPEED_H,
        Speed,
        &dxl_error);
  }
   

  if (dxl_comm_result != COMM_SUCCESS)
  {
    return -1;
  }
  else if (dxl_error != 0)
  {
    return -1;
  }
  else
  {
    return 0;
  }
}

/*Set speed during moves*/
int lib_dynamixel::setSpeed(uint8_t id, uint16_t Speed){
  dxl_error = 0;

  dxl_comm_result = packetHandler->write2ByteTxRx(
        portHandler,
        id,
        AX_GOAL_SPEED_L,
        Speed,
        &dxl_error);
  
   

  if (dxl_comm_result != COMM_SUCCESS)
  {
    return -1;
  }
  else if (dxl_error != 0)
  {
    return -1;
  }
  else
  {
    return 0;
  }
}

int lib_dynamixel::readPosition(uint8_t id){
  // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
    // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
    dxl_comm_result = packetHandler->read2ByteTxRx(
        portHandler,
        id,
        AX_PRESENT_POSITION_L,
        reinterpret_cast<uint16_t *>(&present_position),
        &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    return -1;
  }
  else if (dxl_error != 0)
  {
    return -1;
  }

  return present_position;
}

int lib_dynamixel::readTemperature(uint8_t id){
  // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
    // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
    uint8_t temperature = 0;
    dxl_comm_result = packetHandler->read1ByteTxRx(
        portHandler,
        id,
        AX_PRESENT_TEMPERATURE,
        reinterpret_cast<uint8_t *>(&temperature),
        &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    return -1;
  }
  else if (dxl_error != 0)
  {
    return -1;
  }

  return temperature;
}

int lib_dynamixel::readVoltage(uint8_t id){
  // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
    // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
    uint8_t voltage = 0;
    dxl_comm_result = packetHandler->read1ByteTxRx(
        portHandler,
        id,
        AX_PRESENT_VOLTAGE,
        reinterpret_cast<uint8_t *>(&voltage),
        &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    return -1;
  }
  else if (dxl_error != 0)
  {
    return -1;
  }
  
  return voltage;
}