
#include <arduino.h>
#include "driver/gpio.h"
#include "driver/can.h"
#include "ESP32CAN.h"
/* the variable name CAN_cfg is fixed, do not change */
CAN_device_t CAN_cfg;

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

long atraso;

typedef struct
{
    uint32_t axis_error;
    uint8_t axis_current_state;
    uint8_t xis;
} ODriveHeartbeat;
ODriveHeartbeat heartbeat;

typedef struct
{
    float pos;
    float vel;
} ODriveEnc;
ODriveEnc oEnc;

typedef struct
{
    float Setpoint;
    float Measured;
} ODriveIQ;
ODriveIQ oIQ;

typedef struct
{
    float InputPos;
    int16_t VelFF;
    int16_t TorqueFF;

} ODriveSetInputPos;
ODriveSetInputPos oSetInputPos;

typedef struct
{
    float InputVel;
    float TorqueFF;

} ODriveSetInputVel;
ODriveSetInputVel oSetInputVel;


typedef struct
{
    int32_t ControlMode;
    int32_t InputMode;
} ODriveControllerMode;
ODriveControllerMode oCM;




CAN_frame_t tx_frame;

void sendnow(int axis, int command, bool rtr, byte *bytes, int lengh)
{
    tx_frame.MsgID = ODRIVE_MSG_ID(axis, command);
    tx_frame.FIR.B.DLC = lengh;
    tx_frame.FIR.B.FF = CAN_frame_std;

    if (rtr)
    {
        tx_frame.FIR.B.RTR = CAN_RTR;
    }
    else
    {
        memcpy(tx_frame.data.u8, bytes, sizeof(bytes));
        tx_frame.FIR.B.RTR = CAN_no_RTR;
    }
    ESP32Can.CANWriteFrame(&tx_frame);
    atraso = millis();
}

void setup()
{
    Serial.begin(115200);
    Serial.println("iotsharing.com CAN demo");
    /* set CAN pins and baudrate */
    CAN_cfg.speed = CAN_SPEED_250KBPS;
    CAN_cfg.tx_pin_id = GPIO_NUM_13;
    CAN_cfg.rx_pin_id = GPIO_NUM_12;
    /* create a queue for CAN receiving */
    CAN_cfg.rx_queue = xQueueCreate(10, sizeof(CAN_frame_t));
  
    // initialize CAN Module
    ESP32Can.CANInit();


    int requeststate = 8; //STATE IS CLOSE LOOP
    sendnow(0, ODRIVE_CMD_SET_REQUESTED_STATE, 0, (byte *)&requeststate, 4); //CLOSELOOP
}

long lastsend = 0;

void sendmsg(int interval)
{
    if ((millis() - lastsend) > interval)
    {
        sendnow(0, ODRIVE_CMD_GET_IQ, 1, 0, 0);
        lastsend = millis();
     //   Serial.println("NEW MESSAGE");
   
    }

}

void debug()
{
    Serial.print("HEART: ");
    Serial.print(heartbeat.axis_current_state);
    Serial.print(" ");
    Serial.print(heartbeat.axis_error);
    Serial.print(" ");
    Serial.print(heartbeat.xis);

    Serial.print(" IQ: ");
    Serial.print(oIQ.Measured);
    Serial.print(" ");
    Serial.print(oIQ.Setpoint);
    Serial.print(" ENC: ");

    Serial.print(oEnc.pos);
    Serial.print(" ");
    Serial.print(oEnc.vel);
    Serial.println(" ");

}

void loop()
{
    sendmsg(100);
    CAN_frame_t rx_frame;
    // receive next CAN frame from queue
    if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE)
    {
        if (rx_frame.FIR.B.FF == CAN_frame_std)
        {

        }


        if (rx_frame.FIR.B.RTR == CAN_RTR)
        {

        }else{

            if (ODRIVE_CMD_ID(rx_frame.MsgID) == ODRIVE_CMD_HEARTBEAT)
            {
                heartbeat = *(ODriveHeartbeat *)rx_frame.data.u8;
            }else if (ODRIVE_CMD_ID(rx_frame.MsgID) == ODRIVE_CMD_GET_ENCODER_ESTIMATES)
            {
                oEnc = *(ODriveEnc *)rx_frame.data.u8;

            }else if (ODRIVE_CMD_ID(rx_frame.MsgID) == ODRIVE_CMD_GET_IQ)
            {
            //    Serial.println("NEW MESSAGE");
                oIQ = *(ODriveIQ *)rx_frame.data.u8;

            }
        }
        debug(); //PRINTOUT
    }
}
