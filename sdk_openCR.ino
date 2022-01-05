// control table 참고하라.

#ifdef __linux__
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32MultiArray.h>
#include <DynamixelSDK.h>


// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// See Control Table XL-430
#define ADDR_XL_TORQUE_ENABLE           64  // 1byte
#define ADDR_XL_GOAL_POSITION           116 // 4byte
#define ADDR_XL_GOAL_VELOCITY           116 // 4byte
#define ADDR_XL_PRESENT_POSITION        132  // 4byte

// See which protocol version is used in the Dynamixel
#define PROTOCOL_VERSION2               2.0

// Default setting
#define DXL_ID1    11  // front left
#define DXL_ID2    12  // front right
#define DXL_ID3    13  // back right
#define DXL_ID4    14  // back left
#define DXL_ID5    3  // arm 3
#define DXL_ID6    4  // arm 4
#define DXL_ID7    5  // arm 5
#define DXL_ID8    6  // arm 6
#define DXL_ID9    7  // gripper
#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE     100                 // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE     4000                // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD    10                  // Dynamixel MX moving status threshold

#define ESC_ASCII_VALUE                 0x1b

#define CMD_SERIAL                      Serial

int dxl_id[10] = {DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4, DXL_ID5, DXL_ID6, DXL_ID7, DXL_ID8, DXL_ID9};
int getch();
int kbhit(void);
void velocityCB( const geometry_msgs::Twist& velocity);
void positionCB( const std_msgs::Int32MultiArray& motor_angle);

void setup() {
  Serial.begin(57600);
  while(!Serial); // Wait for Opening Serial Monitor
  Serial.println("Start..");
  
  int index = 0;
  // ros헤더 subscribe 호출
  // opencr init
  nh.initNode();
  //init subscribe node
  nh.subscribe(sub1);
  nh.subscribe(sub2);


  nh.advertise(odom_pub);
  initOdom();
  motor_driver.init("robot");
  prev_update_time = millis();
  
  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  int dxl_comm_result = COMM_TX_FAIL;             // Communication result

  uint8_t dxl_error = 0;                          // Dynamixel error
  uint16_t dxl_model_number;                      // Dynamixel model number

  // Open port
  if (portHandler->openPort())
  {
    Serial.print("Succeeded to open the port!\n");
  }
  else
  {
    Serial.print("Failed to open the port!\n");
    return;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
    Serial.print("Succeeded to change the baudrate!\n");
  }
  else
  {
    Serial.print("Failed to change the baudrate!\n");
    return;
  }

  // Try to ping the Dynamixel
  // Get Dynamixel model number
  index = 0;
  while(index <= 3)
  {
    // Write goal velocity
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_XL_GOAL_VELOCITY, (int32_t)dxl_velocity[index], &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
      Serial.print(packetHandler->getRxPacketError(dxl_error));
    }

    ++index;
  }

  // dynamixel wheelmode
    while(index <= 3){
    //goalVelocity requires motor's round per second (before gear ratio) 258.5/2(pi)
    // Enable Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[index], ADDR_XL_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        Serial.print(packetHandler->getRxPacketError(dxl_error));
    }
    else
    {
        Serial.print("Dynamixel has been successfully connected \n");
    }
    ++index;
  }

  // dynamixel jointMoide
  index = 4
  while(index <= 7)
  {
    Serial.print("Press any key to continue! (or press q to quit!)\n");
    if (getch() == 'q')
      break;

    // Write goal position
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_XL_GOAL_POSITION, (int32_t)motor_angle.data[index], &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
      Serial.print(packetHandler->getRxPacketError(dxl_error));
    }
    ++index;
  }
}

void loop(){
    // velocity, position function

}


int getch()
{
  while(1)
  {
    if( CMD_SERIAL.available() > 0 )
    {
      break;
    }
  }

  return CMD_SERIAL.read();
}

int kbhit(void)
{
  return CMD_SERIAL.available();
}

void velocityCB( const geometry_msgs::Twist& velocity) {
  int index = 0;
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result

  uint8_t dxl_error = 0;                          // Dynamixel error
  int32_t dxl_present_position = 0;               // Present position

  //velocity m/s 
  float speedx = velocity.linear.x;
  float speedy = velocity.linear.y;

  //motor's angular speed rad/s 
  //motor_x+motor_y should not exceed 2(pi)rad/s
  float motor_x = speedx / 0.1;
  float motor_y = speedy / 0.1;

  int32_t dxl_velocity[4] = {(int32_t)(motor_x+motor_y)*41.142, -(int32_t)(motor_x-motor_y)*41.142) , -(int32_t)(motor_x+motor_y)*41.142, (int32_t)(motor_x-motor_y)*41.142};

  while(index <= 3){
    //goalVelocity requires motor's round per second (before gear ratio) 258.5/2(pi)
    // Enable Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[index], ADDR_XL_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        Serial.print(packetHandler->getRxPacketError(dxl_error));
    }
    else
    {
        Serial.print("Dynamixel has been successfully connected \n");
    }
    ++index;
  }
  
  index = 0;
  while(index <= 3)
  {
    Serial.print("Press any key to continue! (or press q to quit!)\n");
    if (getch() == 'q')
      break;

    // Write goal velocity
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_XL_GOAL_VELOCITY, (int32_t)dxl_velocity[index], &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
      Serial.print(packetHandler->getRxPacketError(dxl_error));
    }

    do
    {
      // Read present velocity
      dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_XL_PRESENT_VELOCITY, (uint32_t*)&dxl_present_position, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
      }
      else if (dxl_error != 0)
      {
        Serial.print(packetHandler->getRxPacketError(dxl_error));
      }

      Serial.print("[ID:"); Serial.print(dxl_id[index]);
      Serial.print("] GoalVel:"); Serial.print((int32_t)dxl_velocity[index]);
      Serial.print("  PresVel:"); Serial.print(dxl_present_position);
      Serial.println(" ");

    }while((abs((int32_t)dxl_velocity[index] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD));
    ++index;
  }
  
  index = 0;
  while(index <= 3){
    // Disable Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[index], ADDR_XL_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        Serial.print(packetHandler->getRxPacketError(dxl_error));
    }
    ++index;
  }
}

void positionCB( const std_msgs::Int32MultiArray& motor_angle) {
  int index = 4;
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result

  uint8_t dxl_error = 0;                          // Dynamixel error
  int32_t dxl_present_position = 0;               // Present position
 
  while(index <= 7){
  //goalVelocity requires motor's round per second (before gear ratio) 258.5/2(pi)
  // Enable Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID1, ADDR_XL_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        Serial.print(packetHandler->getRxPacketError(dxl_error));
    }
    else
    {
        Serial.print("Dynamixel has been successfully connected \n");
    }
    ++index;
  }
  
  index = 4
  while(index <= 7)
  {
    Serial.print("Press any key to continue! (or press q to quit!)\n");
    if (getch() == 'q')
      break;

    // Write goal position
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_XL_GOAL_POSITION, (int32_t)motor_angle.data[index], &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
      Serial.print(packetHandler->getRxPacketError(dxl_error));
    }

    do
    {
      // Read present position
      dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_XL_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
      }
      else if (dxl_error != 0)
      {
        Serial.print(packetHandler->getRxPacketError(dxl_error));
      }

      Serial.print("[ID:"); Serial.print(dxl_id[index]);
      Serial.print("] GoalPos:"); Serial.print((int32_t)motor_angle.data[index]);
      Serial.print("  PresPos:"); Serial.print(dxl_present_position);
      Serial.println(" ");

    }while((abs((int32_t)motor_angle.data[index] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD));

    ++index;
  }

  index = 4
  while(index <= 7){
    // Disable Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_XL_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        Serial.print(packetHandler->getRxPacketError(dxl_error));
    }
    ++index;
  }
}