
#include <arduino.h>
#include "driver/gpio.h"
//#include "driver/can.h"
#include "ESP32CAN.h"
#include "ESP_Arduino_CAN.h"
#include "stdio.h"
/* the variable name CAN_cfg is fixed, do not change */

CAN_device_t CAN_cfg;

ESP_Arduino_CAN::ESP_Arduino_CAN()
{

    CAN_cfg.speed = CAN_SPEED_250KBPS;
    CAN_cfg.tx_pin_id = GPIO_NUM_13;
    CAN_cfg.rx_pin_id = GPIO_NUM_12;

    /* create a queue for CAN receiving */
    CAN_cfg.rx_queue = xQueueCreate(10, sizeof(CAN_frame_t));

    // initialize CAN Module
    ESP32Can.CANInit();
}

void ESP_Arduino_CAN::sendMessage(int axis, int command, bool rtr, byte *bytes, int lengh)
{
    CAN_frame_t tx_frame;
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
}

bool ESP_Arduino_CAN::RunStateCloseLoop(int axis_id)
{
    int requeststate = ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL;
    sendMessage(axis_id, ODRIVE_CMD_SET_REQUESTED_STATE, 0, (byte *)&requeststate, 4);
    return true;
}

bool ESP_Arduino_CAN::RunStateIdle(int axis_id)
{
    int requeststate = ODRIVE_AXIS_STATE_IDLE;
    sendMessage(axis_id, ODRIVE_CMD_SET_REQUESTED_STATE, 0, (byte *)&requeststate, 4);
    return true;
}

bool ESP_Arduino_CAN::RunState(int axis_id, int requested_state)
{
    sendMessage(axis_id, ODRIVE_CMD_SET_REQUESTED_STATE, 0, (byte *)&requested_state, 4);
    return true;
}


void ESP_Arduino_CAN::GetVBus(int axis_id)
{
sendMessage(0, ODRIVE_CMD_GET_VBUS_VOLTAGE, true, 0, 0);
}

void ESP_Arduino_CAN::GetIQ(int axis_id)
{
sendMessage(0, ODRIVE_CMD_GET_IQ, true, 0, 0);
}


void ESP_Arduino_CAN::update()
{
    CAN_frame_t rx_frame;
    // receive next CAN frame from queue
    if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE)
    {
        newupdate = 1;
        if (rx_frame.FIR.B.FF == CAN_frame_std) // ID IS
        {
            // DO NOTHING;
        }

        if (rx_frame.FIR.B.RTR == CAN_RTR) // IF IS RTR
        {
            // DO NOTHING;
        }

        else
        {

           // Serial.println(rx_frame.MsgID);
            // DEFINE ID
            uint32_t myid = ODRIVE_AXIS_ID(rx_frame.MsgID);
            

            if (ODRIVE_CMD_ID(rx_frame.MsgID) == ODRIVE_CMD_HEARTBEAT)
            {
                //AXES[myid].hb = *(ODriveHeartbeat *)rx_frame.data.u8;
            }
            else if (ODRIVE_CMD_ID(rx_frame.MsgID) == ODRIVE_CMD_GET_ENCODER_ESTIMATES)
            {
               AXES[myid].Read = *(ODriveEncoderEstimates *)rx_frame.data.u8;
            }
            else if (ODRIVE_CMD_ID(rx_frame.MsgID) == ODRIVE_CMD_GET_IQ)
            {                   
            AXES[myid].IQ = *(ODriveIq *)rx_frame.data.u8;
            }
               else if (ODRIVE_CMD_ID(rx_frame.MsgID) == ODRIVE_CMD_GET_VBUS_VOLTAGE)
            {                   
            Voltage = *(float *)rx_frame.data.u8;
            }
        }
    }
    else
    {
        newupdate = false;
    }
}

void ESP_Arduino_CAN::feedUpdate(uint32_t interval)
{
    if ((millis() - lastfeedupdate) > interval)
    {
        myupdade++;
    GetIQ(0);
        // switch (myupdade)
        // {
        // case 0:
                  
        //     break;
        //         case 1:
        //            GetVBus(0);
        //            myupdade = -1;
        //     break;
        // default:
        //     break;
        // }
      

        lastfeedupdate = millis();
    }
}

void ESP_Arduino_CAN::Debug(int axis_id)
{
    if (newupdate)
    {
        Serial.print(Voltage);
        printf("V AxisState %d E: %d S: %d ", AXES[axis_id].hb.axis_current_state, AXES[axis_id].hb.axis_error, AXES[axis_id].hb.status);
        printf("P: %f V: %f ", AXES[axis_id].Read.position, AXES[axis_id].Read.velocity);
        printf("\n");
        Serial.println( AXES[axis_id].IQ.measured);
        
    }
}