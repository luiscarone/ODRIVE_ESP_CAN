
#include <arduino.h>
#include "driver/gpio.h"
#include "driver/can.h"
//#include "ESP32CAN.h"
#include "ESP_Arduino_CAN.h"
#include "stdio.h"
/* the variable name CAN_cfg is fixed, do not change */



ESP_Arduino_CAN::ESP_Arduino_CAN()
{

can_general_config_t g_config = { // สร้างต้วแปร g_config ใช้กำหนดค่าเกี่ยวกับบัส CAN
    .mode = CAN_MODE_NORMAL,
    .tx_io = GPIO_NUM_13, // กำหนดขา TX ต่อกับ 26
    .rx_io = GPIO_NUM_12, // กำหนดขา TX ต่อกับ 27
    .clkout_io = ((gpio_num_t) - 1),
    .bus_off_io = ((gpio_num_t) - 1),
    .tx_queue_len = 32,
    .rx_queue_len = 32,
    .alerts_enabled = CAN_ALERT_NONE,
    .clkout_divider = 0
  };


  can_timing_config_t t_config = CAN_TIMING_CONFIG_250KBITS();
  can_filter_config_t f_config = CAN_FILTER_CONFIG_ACCEPT_ALL();
  vTaskDelay(1);
  can_driver_install(&g_config, &t_config, &f_config);
  can_start();
}

bool ESP_Arduino_CAN::RunState(int axis_id, int requested_state)
{
    sendMessage(axis_id, ODRIVE_CMD_SET_REQUESTED_STATE, 0, (byte *)&requested_state, 4);
    return true;
}

bool ESP_Arduino_CAN::RunStateIdle(int axis_id)
{
    int requeststate = ODRIVE_AXIS_STATE_IDLE;
    sendMessage(axis_id, ODRIVE_CMD_SET_REQUESTED_STATE, 0, (byte *)&requeststate, 4);
    return true;
}

bool ESP_Arduino_CAN::RunStateCloseLoop(int axis_id)
{
    int requeststate = ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL;
    sendMessage(axis_id, ODRIVE_CMD_SET_REQUESTED_STATE, 0, (byte *)&requeststate, 4);
    return true;
}

void ESP_Arduino_CAN::feedUpdate(uint32_t interval)
{
    if ((millis() - lastfeedupdate) > interval)
    {
        myupdade++;
        //GetIQ(0);
        switch (myupdade)
        {
        case 0:
            GetIQ(0);
            break;
        case 1:
            GetVBus(0);
            myupdade = -1;
            break;
        default:
            myupdade = -1;
            break;
        }

        lastfeedupdate = millis();
    }
}

void ESP_Arduino_CAN::sendMessage(int axis, int command, bool rtr, byte *bytes, int lengh)
{
    can_message_t tx_frame;
    tx_frame.identifier = ODRIVE_MSG_ID(axis, command);
    tx_frame.data_length_code = lengh;
    //tx_frame. = CAN_frame_std;

    if (rtr)
    {
        tx_frame.flags = CAN_MSG_FLAG_RTR;
    }
    else
    {
        memcpy(tx_frame.data, bytes, sizeof(bytes));
        tx_frame.flags = CAN_MSG_FLAG_NONE;
    }
    can_transmit(&tx_frame, 1000);
    for (int i = 0; i < 8; i++)
    {
        /* code */
    }
    
}

// void ESP_Arduino_CAN::update()
// {
//     CAN_frame_t rx_frame;
//     // receive next CAN frame from queue
//     if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE)
//     {
//         newupdate = 1;
//         if (rx_frame.FIR.B.FF == CAN_frame_std) // ID IS
//         {
//             // DO NOTHING;
//         }

//         if (rx_frame.FIR.B.RTR == CAN_RTR) // IF IS RTR
//         {
//             // DO NOTHING;
//         }

//         else
//         {

//             // Serial.println(rx_frame.MsgID);
//             // DEFINE ID
//             uint32_t myid = ODRIVE_AXIS_ID(rx_frame.MsgID);

//             if (ODRIVE_CMD_ID(rx_frame.MsgID) == ODRIVE_CMD_HEARTBEAT)
//             {
//                 // AXES[myid].hb = *(ODriveHeartbeat *)rx_frame.data.u8;
//             }
//             else if (ODRIVE_CMD_ID(rx_frame.MsgID) == ODRIVE_CMD_GET_ENCODER_ESTIMATES)
//             {
//                 AXES[myid].Read = *(ODriveEncoderEstimates *)rx_frame.data.u8;
//             }
//             else if (ODRIVE_CMD_ID(rx_frame.MsgID) == ODRIVE_CMD_GET_IQ)
//             {
//                 AXES[myid].IQ = *(ODriveIq *)rx_frame.data.u8;
//                 // Serial.print("UPDATE;");
//                 // Serial.println(AXES[myid].IQ.measured);
//             }
//             else if (ODRIVE_CMD_ID(rx_frame.MsgID) == ODRIVE_CMD_GET_VBUS_VOLTAGE)
//             {
//                 Voltage = *(float *)rx_frame.data.u8;
//             }
//         }
//     }
//     else
//     {
//         newupdate = false;
//     }
// }

void ESP_Arduino_CAN::SetCtrlModesVelRamp(int axis_id)
{ 
    SetControllerModes(axis_id, ODRIVE_CONTROL_MODE_VELOCITY,ODRIVE_INPUT_MODE_VEL_RAMP);
    
}
void ESP_Arduino_CAN::SetCtrlModePos(int axis_id)
{ 
    SetControllerModes(axis_id, ODRIVE_CONTROL_MODE_POSITION,ODRIVE_INPUT_MODE_POS_FILTER);
}
void ESP_Arduino_CAN::SetControllerModes(int axis_id, int32_t ControlMode, int32_t InputMode)
{ // SetPosition(axis_id, position, velocity_feedforward, 0.0f);
    ODriveControllerModes CtrMode;
    CtrMode.control_mode = ControlMode;
    CtrMode.input_mode = InputMode;
    sendMessage(axis_id, ODRIVE_CMD_SET_CONTROLLER_MODES, false, (byte *)&CtrMode, 8);
}




void ESP_Arduino_CAN::SetPosition(int axis_id, float position)
{
    SetPosition(axis_id, position, 0.0f, 0.0f);
}

void ESP_Arduino_CAN::SetPosition(int axis_id, float position, float velocity_feedforward)
{
    SetPosition(axis_id, position, velocity_feedforward, 0.0f);
}

void ESP_Arduino_CAN::SetPosition(int axis_id, float position, float velocity_feedforward, float current_feedforward)
{ // SetPosition(axis_id, position, velocity_feedforward, 0.0f);
    ODriveInputPosition Setpos;
    Setpos.position = position;
    Setpos.velocity_x1000 = (float)velocity_feedforward / 1000;
    Setpos.torque_x1000 = (float)current_feedforward / 1000;

    sendMessage(axis_id, ODRIVE_CMD_SET_INPUT_POS, false, (byte *)&Setpos, 8);
}

void ESP_Arduino_CAN::SetVelocity(int axis_id, float velocity) {
    SetVelocity(axis_id, velocity, 0.0f);
}

void ESP_Arduino_CAN::SetVelocity(int axis_id, float velocity, float current_feedforward)
{
    ODriveInputVelocity SetVel;
    SetVel.velocity = velocity;
    SetVel.torque = current_feedforward;

    sendMessage(axis_id, 0x00D, false, (byte *)&SetVel, 8);
    // byte* velocity_b = (byte*) &velocity;
    // byte* current_feedforward_b = (byte*) &current_feedforward;
    // byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    // msg_data[0] = velocity_b[0];
    // msg_data[1] = velocity_b[1];
    // msg_data[2] = velocity_b[2];
    // msg_data[3] = velocity_b[3];
    // msg_data[4] = current_feedforward_b[0];
    // msg_data[5] = current_feedforward_b[1];
    // msg_data[6] = current_feedforward_b[2];
    // msg_data[7] = current_feedforward_b[3];
    
    // sendMessage(axis_id, ODRIVE_CMD_SET_INPUT_VEL, false, msgf, 8);

}

void ESP_Arduino_CAN::SetTorque(int axis_id, float torque)
{
    byte *torque_b = (byte *)&torque;

    sendMessage(axis_id, ODRIVE_CMD_SET_INPUT_TORQUE, false, torque_b, 4);
}

void ESP_Arduino_CAN::GetVBus(int axis_id)
{
    sendMessage(0, ODRIVE_CMD_GET_VBUS_VOLTAGE, true, 0, 0);
}

void ESP_Arduino_CAN::GetIQ(int axis_id)
{
    sendMessage(0, ODRIVE_CMD_GET_IQ, true, 0, 0);
}

void ESP_Arduino_CAN::Debug(int axis_id)
{
    if (newupdate)
    {
        //======================================================================================================================
        /* Serial.print(Voltage);
        printf("V AxisState %d E: %d S: %d ", AXES[axis_id].hb.axis_current_state, AXES[axis_id].hb.axis_error, AXES[axis_id].hb.status);
        printf("P: %f V: %f ", AXES[axis_id].Read.position, AXES[axis_id].Read.velocity);
        printf("\n");
        Serial.println( AXES[axis_id].IQ.measured); */
        //=======================================================================================================================
    }
}