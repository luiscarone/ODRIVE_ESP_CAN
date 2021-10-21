#include <Arduino.h>
#include <ESP32CAN.h>
#include <CAN_config.h>

// refer to https://docs.odriverobotics.com/can-protocol
#define ODRIVE_MSG_ID(AXIS_ID, CMD_ID) ((AXIS_ID << 5) | CMD_ID)
#define ODRIVE_AXIS_ID(MSG_ID) (MSG_ID >> 5)
#define ODRIVE_CMD_ID(MSG_ID) (MSG_ID & 0x1F)

// Command IDs
#define ODRIVE_CMD_CANOPEN_NMT 0x00
#define ODRIVE_CMD_HEARTBEAT 0x01
#define ODRIVE_CMD_ESTOP 0x02
#define ODRIVE_CMD_GET_MOTOR_ERROR 0x03
#define ODRIVE_CMD_GET_ENCODER_ERROR 0x04
#define ODRIVE_CMD_GET_SENSORLESS_ERROR 0x05
#define ODRIVE_CMD_SET_AXIS_NODE_ID 0x06
#define ODRIVE_CMD_SET_REQUESTED_STATE 0x07
#define ODRIVE_CMD_SET_STARTUP_CONFIG 0x08
#define ODRIVE_CMD_GET_ENCODER_ESTIMATES 0x09
#define ODRIVE_CMD_GET_ENCODER_COUNT 0x0A
#define ODRIVE_CMD_SET_CONTROLLER_MODES 0x0B
#define ODRIVE_CMD_SET_INPUT_POS 0x0C
#define ODRIVE_CMD_SET_INPUT_VEL 0x0D
#define ODRIVE_CMD_SET_INPUT_TORQUE 0x0E
#define ODRIVE_CMD_SET_LIMITS 0x0F
#define ODRIVE_CMD_START_ANTICOGGING 0x10
#define ODRIVE_CMD_SET_TRAJ_VEL_LIMIT 0x11
#define ODRIVE_CMD_SET_TRAJ_ACCEL_LIMITs 0x12
#define ODRIVE_CMD_SET_TRAJ_INERTIA 0x13
#define ODRIVE_CMD_GET_IQ 0x14
#define ODRIVE_CMD_GET_SENSORLESS_ESTIMATES 0x15
#define ODRIVE_CMD_REBOOT 0x16
#define ODRIVE_CMD_GET_VBUS_VOLTAGE 0x17
#define ODRIVE_CMD_CLEAR_ERRORS 0x18
#define ODRIVE_CMD_SET_LINEAR_COUNT 0x19
#define ODRIVE_CMD_CANOPEN_HEARTBEAT 0x700

typedef struct
{
  uint32_t axis_error;
  uint8_t axis_current_state;
  uint8_t xis;
} ODriveHeartbeat;

ODriveHeartbeat heartbeat;

CAN_device_t CAN_cfg;             // CAN Config
unsigned long previousMillis = 0; // will store last time a CAN Message was send
const int interval = 80;          // interval at which send CAN Messages (milliseconds)
const int rx_queue_size = 32;     // Receive Queue size

void setup()
{
  Serial.begin(115200);
  Serial.println("Basic Demo - ESP32-Arduino-CAN");
  CAN_cfg.speed = CAN_SPEED_250KBPS;
  CAN_cfg.tx_pin_id = GPIO_NUM_13;
  CAN_cfg.rx_pin_id = GPIO_NUM_12;
  CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));
  // Init CAN Module
  ESP32Can.CANInit();
}
byte data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
CAN_frame_t rx_frame;
CAN_frame_t tx_frame;
void loop()
{
  for (int32_t i = 0; i <= 10000; i += 10)
  {
    // tx_frame.MsgID = 0x000 << 5 | 0x00D;
    // //  uint32_t can_id = 0x001 << 5 + 0x017;
    // // x_frame.data.u8[0] = 0x00;
    // //    tx_frame.data.u8[1] = 0x01;
    // //    tx_frame.data.u8[2] = 0x02;
    // //    tx_frame.data.u8[3] = 0x03;
    // //    tx_frame.data.u8[4] = 0x04;
    // //    tx_frame.data.u8[5] = 0x05;
    // //    tx_frame.data.u8[6] = 0x06;
    // //    tx_frame.data.u8[7] = 0x07;

    // //  tx_frame.FIR.B.DLC = 8;
    // //   tx_frame.FIR.B.RTR = CAN_RTR;
    // //   tx_frame.FIR.B.FF = CAN_frame_std;
    // tx_frame.data.u8[0] = (int32_t)i & 0xFF;
    // tx_frame.data.u8[1] = (int32_t)(i >> 8) & 0xFF;
    // tx_frame.data.u8[2] = (int32_t)(i >> 16) & 0xFF;
    // tx_frame.data.u8[3] = (int32_t)(i >> 24) & 0xFF;
    // tx_frame.data.u8[4] = (int32_t)00 & 0xFF;
    // tx_frame.data.u8[5] = (int32_t)(00 >> 8) & 0xFF;
    // // send data:  ID = 0x100, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send

    // ESP32Can.CANWriteFrame(&tx_frame);

    if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE)
    {

      if (rx_frame.FIR.B.FF == CAN_frame_std)
      {
        Serial.print("Recive");
        Serial.println(millis());
        printf("New standard frame");
      }
      else
      {
        printf("New extended frame");
      }

      if (rx_frame.FIR.B.RTR == CAN_RTR)
      {
        printf(" RTR from 0x%08X, DLC %d\r\n", rx_frame.MsgID, rx_frame.FIR.B.DLC);
      }
      else
      {
        printf(" from 0x%08X, DLC %d, Data ", rx_frame.MsgID, rx_frame.FIR.B.DLC);
        for (int i = 0; i < rx_frame.FIR.B.DLC; i++)
        {
          printf("0x%02X ", rx_frame.data.u8[i]);
        }
        heartbeat = *(ODriveHeartbeat *)rx_frame.data.u8;
        Serial.print("AXIS: ");
        Serial.println(ODRIVE_AXIS_ID(rx_frame.MsgID), HEX);
        Serial.print("CMD: ");

        Serial.println(ODRIVE_CMD_ID(rx_frame.MsgID), HEX);

        if (ODRIVE_CMD_ID(rx_frame.MsgID) == ODRIVE_CMD_HEARTBEAT)
        {

          Serial.println(heartbeat.axis_current_state);
          Serial.println(heartbeat.axis_error);
          Serial.println(heartbeat.xis);
        }
        printf("\n");
      }
    }
  }
}