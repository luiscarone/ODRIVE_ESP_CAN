
#include <arduino.h>
#include "driver/gpio.h"
#include "driver/can.h"

#include "ESP_Arduino_CAN.h"

#include "stdio.h"

can_general_config_t g_config =
    {
        .mode = CAN_MODE_NORMAL,
        .tx_io = GPIO_NUM_4,
        .rx_io = GPIO_NUM_5,
        .clkout_io = ((gpio_num_t)-1),
        .bus_off_io = ((gpio_num_t)-1),
        .tx_queue_len = 32,
        .rx_queue_len = 32,
        .alerts_enabled = CAN_ALERT_ALL,
        .clkout_divider = 0};

ESP_Arduino_CAN::ESP_Arduino_CAN(gpio_num_t tx, gpio_num_t rx)
{
    g_config.tx_io = tx;
    g_config.rx_io = rx;

    can_timing_config_t t_config = CAN_TIMING_CONFIG_250KBITS();
    can_filter_config_t f_config = CAN_FILTER_CONFIG_ACCEPT_ALL();

    if (can_driver_install(&g_config, &t_config, &f_config) != ESP_OK)
    {
        Serial.println("Failed to install driver");
        return;
        
    }

    if (can_start() != ESP_OK)
    {
        Serial.println("Failed to start driver");
        return;
    }
}

// CAN REQUEST

void ESP_Arduino_CAN::sendMessage(int axis, int command, bool rtr, byte *bytes, int lengh)
{
    can_message_t tx_frame;
    tx_frame.identifier = ODRIVE_MSG_ID(axis, command);
    tx_frame.data_length_code = lengh;

    if (rtr)
    {
        tx_frame.flags = CAN_MSG_FLAG_RTR;
    }
    else
    {
        memcpy(tx_frame.data, bytes, lengh);
        tx_frame.flags = CAN_MSG_FLAG_NONE;
    }
    if (can_transmit(&tx_frame, pdMS_TO_TICKS(1000)) == ESP_OK)
    {
        CanCheck = false;
        // Serial.println("Message queued for transmission");
    }
    else
    {
        CanCheck = true;

        Serial.println("Failed to queue message for transmission");
    }
}
void ESP_Arduino_CAN::update()
{
    can_message_t rx_frame;
    // receive next CAN frame from queue

    // for (TickType_t tick = xTaskGetTickCount();; vTaskDelayUntil(&tick, 1)){
    if (can_receive(&rx_frame, 0) == ESP_OK)
    {

        if (rx_frame.flags == CAN_MSG_FLAG_RTR) // IF IS RTR
        {
            // DO NOTHING;
        }

        uint32_t myid = ODRIVE_AXIS_ID(rx_frame.identifier);
        uint32_t CMD = ODRIVE_CMD_ID(rx_frame.identifier);
        if (myid <= NUMOFAXIS){
            
            newupdate = true;

        switch (CMD)
        {
        case ODRIVE_CMD_HEARTBEAT:
            AXES[myid].hb = *(ODriveHeartbeat *)rx_frame.data;
            break;

        case ODRIVE_CMD_GET_ENCODER_ESTIMATES:
            AXES[myid].Read = *(ODriveEncoderEstimates *)rx_frame.data;
            break;

        case ODRIVE_CMD_GET_IQ:
            AXES[myid].IQ = *(ODriveIq *)rx_frame.data;
            break;

        case ODRIVE_CMD_GET_VBUS_VOLTAGE:
            AXES[myid].Voltage = *(float *)rx_frame.data;
            break;

        case ODRIVE_CMD_GET_MYDATA:
            dataread = *(OdriveMyData *)rx_frame.data;
            AXES[myid].CM.control_mode = dataread.control_mode;
            AXES[myid].CM.input_mode = dataread.input_mode;
            AXES[myid].IQ.measured = float(dataread.Iq_measured) / 100;
            AXES[myid].Fet_Temperature = (float)dataread.fet_temperature / 2;
            AXES[myid].Voltage =  float(dataread.vbus_voltage) / 100;
            AXES[myid].TrajDone = dataread.trajectory_done;
            break;

        default:
            break;
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

        can_status_info_t status_info;
        can_get_status_info(&status_info);

            if(status_info.msgs_to_rx < 28)

            GetMyData(0);
        
        lastfeedupdate = millis();
    }
}

// CAN OPERATIONS CALLED FROM OUTSIDE
void ESP_Arduino_CAN::Can_clear_rx_queue()
{
    can_clear_receive_queue();
}
void ESP_Arduino_CAN::Can_clear_tx_queue()
{
    can_clear_transmit_queue();
}

// AXIS REQUEST STATE
bool ESP_Arduino_CAN::RunState(int axis_id, int requested_state)
{
    can_clear_transmit_queue();
    can_clear_receive_queue();
    sendMessage(axis_id, ODRIVE_CMD_SET_REQUESTED_STATE, 0, (byte *)&requested_state, 4);
    return true;
}
bool ESP_Arduino_CAN::RunStateIdle(int axis_id)
{
    RunState(axis_id, ODRIVE_AXIS_STATE_IDLE);

    return true;
}
bool ESP_Arduino_CAN::RunStateCloseLoop(int axis_id)
{

    RunState(axis_id, ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL);
    return true;
}

// CONTROLMODES

void ESP_Arduino_CAN::autoCtrlMode(int axis_id, ODriveControllerModes _RCM)
{
    if (_RCM.control_mode != AXES[axis_id].CM.control_mode or _RCM.input_mode != AXES[axis_id].CM.input_mode)
    {
        can_clear_receive_queue();
        can_clear_transmit_queue();
        if (_RCM.control_mode == ODRIVE_CONTROL_MODE_POSITION)
        {
            SetPosReset(axis_id);
        }
        SetCtrlMode(axis_id, _RCM.control_mode, _RCM.input_mode);
    }
}
void ESP_Arduino_CAN::SetCtrlMode(int axis_id, int ControlMode, int InputMode)
{

    ODriveControllerModes CtrlModess;
    CtrlModess.control_mode = ControlMode;
    CtrlModess.input_mode = InputMode;
    sendMessage(axis_id, ODRIVE_CMD_SET_CONTROLLER_MODES, false, (byte *)&CtrlModess, 8);
}

// POSITION

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
{
    ODriveInputPosition Setpos;
    Setpos.position = AXES[axis_id].Read.position;
    Setpos.velocity_x1000 = 0.01;
    Setpos.torque_x1000 = 0.01;
    sendMessage(axis_id, ODRIVE_CMD_SET_INPUT_POS, false, (byte *)&Setpos, 8);
    Serial.println("ZEREIln///////////////////////////////////////////////////////");
    ODriveControllerModes pass{ODRIVE_CONTROL_MODE_VELOCITY, ODRIVE_INPUT_MODE_PASSTHROUGH};

    SetCtrlMode(axis_id, ODRIVE_CONTROL_MODE_VELOCITY, ODRIVE_INPUT_MODE_PASSTHROUGH);
    while (AXES[axis_id].CM.control_mode != pass.control_mode or AXES[axis_id].CM.input_mode != pass.input_mode)
    {
        update();
        GetMyData(axis_id);
        vTaskDelay(2);
        Serial.println("NOT YET");
    }
    vTaskDelay(5);
    Serial.println("TRANSOK!////////////////////////////////////////////////////");
}

// TRAJECTORY
void ESP_Arduino_CAN::SetTrajPos(int axis_id, float position)
{
    autoCtrlMode(axis_id, {ODRIVE_CONTROL_MODE_POSITION, ODRIVE_INPUT_MODE_TRAP_TRAJ});

    ODriveInputPosition Setpos;
    // vTaskDelay(5);
    Setpos.position = position;
    Setpos.velocity_x1000 = 0;
    Setpos.torque_x1000 = 0;
    sendMessage(axis_id, ODRIVE_CMD_SET_INPUT_POS, false, (byte *)&Setpos, 8);
}
void ESP_Arduino_CAN::setTrajVel(int axis_id, float _vel)
{
    sendMessage(axis_id, ODRIVE_CMD_SET_TRAJ_VEL_LIMIT, false, (byte *)&_vel, 4);
}
void ESP_Arduino_CAN::setTrajaccel(int axis_id, float _acc, float _dec)
{

    AXES[axis_id].TL.accel_limit = _acc;
    AXES[axis_id].TL.decel_limit = _dec;

    sendMessage(axis_id, ODRIVE_CMD_SET_TRAJ_ACCEL_LIMITS, false, (byte *)&AXES[axis_id].TL, 8);
}
void ESP_Arduino_CAN::setTrajacc(int axis_id, float _acc)
{

    AXES[axis_id].TL.accel_limit = _acc;

    sendMessage(axis_id, ODRIVE_CMD_SET_TRAJ_ACCEL_LIMITS, false, (byte *)&AXES[axis_id].TL, 8);

    Serial.print("_>>> ACC:");
    Serial.print(AXES[axis_id].TL.accel_limit);
    Serial.print(" DEC:");
    Serial.println(AXES[axis_id].TL.decel_limit);
}
void ESP_Arduino_CAN::setTrajdec(int axis_id, float _dec)
{

    AXES[axis_id].TL.decel_limit = _dec;

    sendMessage(axis_id, ODRIVE_CMD_SET_TRAJ_ACCEL_LIMITS, false, (byte *)&AXES[axis_id].TL, 8);
    Serial.print(AXES[axis_id].TL.accel_limit);
    Serial.println(AXES[axis_id].TL.decel_limit);
}

// VELOCITY

void ESP_Arduino_CAN::SetVelocity(int axis_id, float velocity)
{
    SetVelocity(axis_id, velocity, 0.0f);
}
void ESP_Arduino_CAN::SetVelocity(int axis_id, float velocity, float current_feedforward)
{
    autoCtrlMode(axis_id, {ODRIVE_CONTROL_MODE_VELOCITY, ODRIVE_INPUT_MODE_VEL_RAMP});

    ODriveInputVelocity SetVel;
    SetVel.velocity = velocity;
    SetVel.torque = current_feedforward;

    sendMessage(axis_id, ODRIVE_CMD_SET_INPUT_VEL, false, (byte *)&SetVel, 8);
}

void ESP_Arduino_CAN::SetLimits(int axis_id, float velLimit, float curLimit)
{
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

/// PID
void ESP_Arduino_CAN::SetPositionGains(int axis_id, float pos_gain)
{
    sendMessage(axis_id, ODRIVE_CMD_VEL_GAINS, false, (byte *)&pos_gain, 4);
}

void ESP_Arduino_CAN::SetVelGains(int axis_id, ODriveVelGains VelGains)
{
    sendMessage(axis_id, ODRIVE_CMD_VEL_GAINS, false, (byte *)&VelGains, 8);
}
void ESP_Arduino_CAN::SetVel_Int_Gain(int axis_id, float VelIntGain)
{

    AXES[axis_id].VG.VelIntegratorGain = VelIntGain;
    SetVelGains(axis_id, AXES[axis_id].VG);
}
void ESP_Arduino_CAN::SetVel_Gain(int axis_id, float VelGain)
{
        AXES[axis_id].VG.VelGain = VelGain;
        SetVelGains(axis_id, AXES[axis_id].VG);
}

// ODRIVE OPERATIONS
void ESP_Arduino_CAN::Reboot(int axis_id)
{
        Can_clear_rx_queue();
    Can_clear_tx_queue();
    sendMessage(axis_id, ODRIVE_CMD_REBOOT, false, 0, 0);
}
void ESP_Arduino_CAN::Save_configuration(int axis_id)
{
    // ALL AXIS SHOULD BE IN IDLE to save..
    RunStateIdle(axis_id);
    sendMessage(axis_id, ODRIVE_CMD_SAVE_CONFIGURATION, false, 0, 0);
}
void ESP_Arduino_CAN::ClearErrors(int axis_id)
{
    Can_clear_rx_queue();
    Can_clear_tx_queue();
    sendMessage(axis_id, (ODRIVE_CMD_CLEAR_ERRORS), false, 0, 0);
}

void ESP_Arduino_CAN::GetVBus(int axis_id)
{
    sendMessage(axis_id, ODRIVE_CMD_GET_VBUS_VOLTAGE, true, 0, 0);
}
void ESP_Arduino_CAN::GetIQ(int axis_id)
{
    sendMessage(axis_id, ODRIVE_CMD_GET_IQ, true, 0, 0);
}
void ESP_Arduino_CAN::GetMyData(int axis_id)
{
    sendMessage(axis_id, ODRIVE_CMD_GET_MYDATA, true, 0, 0);
}

//DEBUG
void ESP_Arduino_CAN::Debug(int axis_id)
{
    if (newupdate)
    {
        //======================================================================================================================
        Serial.print( AXES[axis_id].Voltage);
        printf("Axis:%d State %d E: %d S: %d ", axis_id,  AXES[axis_id].hb.axis_current_state, AXES[axis_id].hb.axis_error, AXES[axis_id].hb.status);
        printf(" cm: %d input:%d ", AXES[axis_id].CM.control_mode ,AXES[axis_id].CM.input_mode) ;
        printf(" T_Done: %d " , AXES[axis_id].TrajDone);
        printf("P: %.1f V: %.1f ", AXES[axis_id].Read.position, AXES[axis_id].Read.velocity);
        printf(" IQ: %.1f Fet:%.1f \n", AXES[axis_id].IQ.measured,AXES[axis_id].Fet_Temperature);


        //=======================================================================================================================
    }
}
void ESP_Arduino_CAN::DebugMyData()
{
    if (newupdate)
    {
        Serial.print("vbus: ");
        Serial.print(float(dataread.vbus_voltage) / 100);
        Serial.print(" Iq: ");
        Serial.print(float(dataread.Iq_measured) / 100);
        Serial.print(" tra_done: ");
        Serial.print(dataread.trajectory_done);
        Serial.print(" mode: ");
        Serial.print(dataread.control_mode);
        Serial.print(" input: ");
        Serial.print(dataread.input_mode);
        Serial.print(" fet: ");
        Serial.print((float)dataread.fet_temperature / 2);
        Serial.println();
    }
}
void ESP_Arduino_CAN::CanDebug()
{
    if (newupdate)
    {        
        can_status_info_t status_info;
        can_get_status_info(&status_info);
        // Serial.print(" s:");
        // Serial.print(status_info.state);
        // Serial.print(" tx:");
        // Serial.print(status_info.msgs_to_tx);
        // Serial.print(" x:");
        Serial.println(status_info.msgs_to_rx);
        // Serial.print(" x:");
        // Serial.print(status_info.tx_error_counter);
        // Serial.print(" rx_e:");
        // Serial.print(status_info.rx_error_counter);
        // Serial.print(" tx_c:");
        // Serial.print(status_info.tx_failed_count);
        // Serial.print(" rx_c:");
        // Serial.print(status_info.rx_missed_count);
        // Serial.print(" air_c:");
        // Serial.print(status_info.arb_lost_count);
        // Serial.print(" bus_c:");
        // Serial.print(status_info.bus_error_count);
        // Serial.println();
    }
}

void ESP_Arduino_CAN::CanDebugFast(int interval)
{
if (newupdate)
    {
        can_status_info_t status_info;
        can_get_status_info(&status_info);
        // Serial.print(" s:");
        // Serial.print(status_info.state);
        // Serial.print(" tx:");
        Serial.print(status_info.msgs_to_tx);
        Serial.print("t");
        Serial.print(status_info.msgs_to_rx);
        Serial.print(" x");
        // Serial.print(status_info.tx_error_counter);
        // Serial.print(" rx_e:");
        // Serial.print(status_info.rx_error_counter);
        // Serial.print(" tx_c:");
        // Serial.print(status_info.tx_failed_count);
        // Serial.print(" rx_c:");
        // Serial.print(status_info.rx_missed_count);
        // Serial.print(" air_c:");
        // Serial.print(status_info.arb_lost_count);
        // Serial.print(" bus_c:");
        // Serial.print(status_info.bus_error_count);
        Serial.println();
    }
}
