// control table 참고하라.
#include <DynamixelSDK.h>

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// See Control Table XL-430
#define ADDR_XL_TORQUE_ENABLE           64   // 1byte
#define ADDR_XL_GOAL_POSITION           116  // 4byte
#define ADDR_XL_GOAL_VELOCITY           104  // 4byte
#define ADDR_XL_PRESENT_POSITION        132  // 4byte
#define ADDR_XL_PRESENT_VELOCITY        128  // 4byte

// See which protocol version is used in the Dynamixel
#define PROTOCOL_VERSION2               2.0

// Default setting
#define DXL_ID    1  // front left
#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"
#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_VELOCITY_VALUE     100                 // Dynamixel will rotate between this value
#define DXL_MAXIMUM_VELOCITY_VALUE     4000                // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD    10                  // Dynamixel MX moving status threshold

#define ESC_ASCII_VALUE                 0x1b

#define CMD_SERIAL                      Serial

int getch();
int kbhit(void);

Serial.begin(57600);
while(!Serial); // Wait for Opening Serial Monitor
Serial.println("Start..");

// Initialize PortHandler instance
// Set the port path
// Get methods and members of PortHandlerLinux or PortHandlerWindows
dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

// Initialize PacketHandler instance
// Set the protocol version
// Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

int dxl_comm_result = COMM_TX_FAIL;             // Communication result
int index = 0;
int dxl_comm_result = COMM_TX_FAIL;             // Communication result
int dxl_goal_velocity[2] = {DXL_MINIMUM_VELOCITY_VALUE, DXL_MAXIMUM_VELOCITY_VALUE};         // Goal position
uint8_t dxl_error = 0;                          // Dynamixel error
 int32_t dxl_present_velocity = 0;               // Present velocity
uint16_t dxl_model_number;                      // Dynamixel model number



void setup() {
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

    // dynamixel wheelmode
    // Enable Dynamixel Torque
    //goalVelocity requires motor's round per second (before gear ratio) 258.5/2(pi)
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_XL_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
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

void loop(){
      // Enable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_XL_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
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

  while(1)
  {
    Serial.print("Press any key to continue! (or press q to quit!)\n");
    if (getch() == 'q')
      break;

    // Write goal position
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_XL_GOAL_VELOCITY, dxl_goal_velocity[index], &dxl_error);
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
      dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_XL_PRESENT_POSITION, (uint32_t*)&dxl_present_velocity, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
      }
      else if (dxl_error != 0)
      {
        Serial.print(packetHandler->getRxPacketError(dxl_error));
      }

      Serial.print("[ID:"); Serial.print(DXL_ID);
      Serial.print("] GoalPos:"); Serial.print(dxl_goal_velocity[index]);
      Serial.print("  PresPos:"); Serial.print(dxl_present_velocity);
      Serial.println(" ");

    }while((abs(dxl_goal_velocity[index] - dxl_present_velocity) > DXL_MOVING_STATUS_THRESHOLD));

    // Change goal position
    if (index == 0)
    {
      index = 1;
    }
    else
    {
      index = 0;
    }
  }

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