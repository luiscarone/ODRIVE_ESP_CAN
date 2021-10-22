
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
        memcpy(tx_frame.data.u8, bytes, lengh);
        tx_frame.FIR.B.RTR = CAN_no_RTR;
    }


    ESP32Can.CANWriteFrame(&tx_frame);
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
                AXES[myid].hb = *(ODriveHeartbeat *)rx_frame.data.u8;
            }
            else if (ODRIVE_CMD_ID(rx_frame.MsgID) == ODRIVE_CMD_GET_ENCODER_ESTIMATES)
            {
                AXES[myid].Read = *(ODriveEncoderEstimates *)rx_frame.data.u8;
            }
            else if (ODRIVE_CMD_ID(rx_frame.MsgID) == ODRIVE_CMD_GET_IQ)
            {
                AXES[myid].IQ = *(ODriveIq *)rx_frame.data.u8;
                // Serial.print("UPDATE;");
                // Serial.println(AXES[myid].IQ.measured);
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

void ESP_Arduino_CAN::SetCtrlModesVelRamp(int axis_id)
{ 
    SetCtrlMode(axis_id, ODRIVE_CONTROL_MODE_VELOCITY,ODRIVE_INPUT_MODE_VEL_RAMP);
}
void ESP_Arduino_CAN::SetCtrlModePos(int axis_id)
{ 
    SetCtrlMode(axis_id, ODRIVE_CONTROL_MODE_POSITION,ODRIVE_INPUT_MODE_POS_FILTER);
}
void ESP_Arduino_CAN::SetCtrlMode(int axis_id, int ControlMode, int InputMode)
{ // SetPosition(axis_id, position, velocity_feedforward, 0.0f);
    // ODriveControllerModes CtrMode;
    // CtrMode.control_mode = ControlMode;
    // CtrMode.input_mode = InputMode;

    byte* velocity_b = (byte*) &ControlMode;
    byte* current_feedforward_b = (byte*) &InputMode;
    byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    msg_data[0] = velocity_b[0];
    msg_data[1] = velocity_b[1];
    msg_data[2] = velocity_b[2];
    msg_data[3] = velocity_b[3];
    msg_data[4] = current_feedforward_b[0];
    msg_data[5] = current_feedforward_b[1];
    msg_data[6] = current_feedforward_b[2];
    msg_data[7] = current_feedforward_b[3];



    sendMessage(axis_id, ODRIVE_CMD_SET_CONTROLLER_MODES, false, msg_data, 8);
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
    
    Setpos.velocity_x1000 = (velocity_feedforward * 1000);
    Setpos.torque_x1000 = (current_feedforward * 1000);
    Serial.println(Setpos.velocity_x1000);

    sendMessage(axis_id, ODRIVE_CMD_SET_INPUT_POS, false, (byte *)&Setpos, 8);
    autoCtrlMode(axis_id, {ODRIVE_CONTROL_MODE_POSITION, ODRIVE_INPUT_MODE_PASSTHROUGH});

}

void ESP_Arduino_CAN::SetPosReset(int axis_id)
{ // SetPosition(axis_id, position, velocity_feedforward, 0.0f);
    autoCtrlMode(axis_id, {ODRIVE_CONTROL_MODE_VELOCITY, ODRIVE_INPUT_MODE_PASSTHROUGH});

    ODriveInputPosition Setpos;
    Setpos.position = AXES[axis_id].Read.position;
    
    Setpos.velocity_x1000 = 0;
    Setpos.torque_x1000 = 0;
    sendMessage(axis_id, ODRIVE_CMD_SET_INPUT_POS, false, (byte *)&Setpos, 8);
}
void ESP_Arduino_CAN::SetVelocity(int axis_id, float velocity) {
    SetVelocity(axis_id, velocity, 0.0f);
}

void ESP_Arduino_CAN::autoCtrlMode(int axis_id, ODriveControllerModes _RCM){
    if(_RCM.control_mode != AXES[axis_id].CM.control_mode || _RCM.input_mode != AXES[axis_id].CM.input_mode){
        AXES[axis_id].CM.control_mode = _RCM.control_mode; 
        AXES[axis_id].CM.input_mode = _RCM.input_mode;
        SetCtrlMode(axis_id, _RCM.control_mode, _RCM.input_mode);
    }
}

void ESP_Arduino_CAN::SetVelocity(int axis_id, float velocity, float current_feedforward)
{   
    autoCtrlMode(axis_id, {ODRIVE_CONTROL_MODE_VELOCITY, ODRIVE_INPUT_MODE_VEL_RAMP});

    ODriveInputVelocity SetVel;
    SetVel.velocity = velocity;
    SetVel.torque = current_feedforward;

    sendMessage(axis_id, ODRIVE_CMD_SET_INPUT_VEL, false, (byte *)&SetVel, 8);

}

void ESP_Arduino_CAN::setTrajVel(int axis_id, float _vel)
{
    sendMessage(axis_id, ODRIVE_CMD_SET_TRAJ_VEL_LIMIT, false, (byte *)&_vel, 4);
}
void ESP_Arduino_CAN::setTrajaccel(int axis_id, float _acc, float _dec)
{
   
   AXES[axis_id].TL.accel_limit =_acc;
   AXES[axis_id].TL.decel_limit =_dec;

    // ODriveTrajAccelLimit trajLimits;
    // trajLimits.accel_limit = _acc;
    // trajLimits.decel_limit = _dec;

    sendMessage(axis_id, ODRIVE_CMD_SET_TRAJ_ACCEL_LIMITS, false, (byte *)&AXES[axis_id].TL, 8);
}

void ESP_Arduino_CAN::setTrajacc(int axis_id, float _acc)
{
   
   AXES[axis_id].TL.accel_limit =_acc;

    sendMessage(axis_id, ODRIVE_CMD_SET_TRAJ_ACCEL_LIMITS, false, (byte *)&AXES[axis_id].TL, 8);
}


void ESP_Arduino_CAN::setTrajdec(int axis_id, float _dec)
{
   
   AXES[axis_id].TL.decel_limit =_dec;

    sendMessage(axis_id, ODRIVE_CMD_SET_TRAJ_ACCEL_LIMITS, false, (byte *)&AXES[axis_id].TL, 8);
}

void ESP_Arduino_CAN::SetTrajPos(int axis_id, float position)
{
    SetPosReset(axis_id);
    autoCtrlMode(axis_id, {ODRIVE_CONTROL_MODE_POSITION, ODRIVE_INPUT_MODE_TRAP_TRAJ});
    ODriveInputPosition Setpos;

    //vTaskDelay(5);
    Setpos.position = position;
    
    Setpos.velocity_x1000 = 0;
    Setpos.torque_x1000 = 0;

    sendMessage(axis_id, ODRIVE_CMD_SET_INPUT_POS, false, (byte *)&Setpos, 8);
}

void ESP_Arduino_CAN::SetLimits(int axis_id, float velLimit, float curLimit){
    ODriveInputLimits limits;
    limits.velocity_limit = velLimit;
    limits.current_limit = curLimit;
    sendMessage(axis_id, ODRIVE_CMD_SET_LIMITS, false, (byte *)&limits, 8);
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
         Serial.print(Voltage);
        printf("V AxisState %d E: %d S: %d ", AXES[axis_id].hb.axis_current_state, AXES[axis_id].hb.axis_error, AXES[axis_id].hb.status);
        printf("P: %f V: %f ", AXES[axis_id].Read.position, AXES[axis_id].Read.velocity);
        printf(" IQ: %f\n",  AXES[axis_id].IQ.measured); 
        //=======================================================================================================================
    }
}