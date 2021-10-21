#include "Arduino.h"
#include "ODriveTeensyCAN.h"
#include "driver/gpio.h"
#include "driver/can.h"

static const int kMotorOffsetFloat = 2;
static const int kMotorStrideFloat = 28;
static const int kMotorOffsetInt32 = 0;
static const int kMotorStrideInt32 = 4;
static const int kMotorOffsetBool = 0;
static const int kMotorStrideBool = 4;
static const int kMotorOffsetUint16 = 0;
static const int kMotorStrideUint16 = 2;

static const int NodeIDLength = 6;
static const int CommandIDLength = 5;

static const float feedforwardFactor = 1 / 0.001;

static const int CANBaudRate = 250000;

// Print with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

//FlexCAN_T4<CAN0, RX_SIZE_256, TX_SIZE_16> Can0;
// FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can0;

ODriveTeensyCAN::ODriveTeensyCAN() {
    can_general_config_t g_config = { // สร้างต้วแปร g_config ใช้กำหนดค่าเกี่ยวกับบัส CAN
    .mode = CAN_MODE_NORMAL,
    .tx_io = GPIO_NUM_13, // กำหนดขา TX ต่อกับ 26
    .rx_io = GPIO_NUM_12, // กำหนดขา TX ต่อกับ 27
    .clkout_io = ((gpio_num_t) - 1),
    .bus_off_io = ((gpio_num_t) - 1),
    .tx_queue_len = 32,
    .rx_queue_len = 60,
    .alerts_enabled = CAN_ALERT_ALL,
    .clkout_divider = 0
  };
  
    //g_config.intr_flags = ESP_INTR_FLAG_IRAM;
    g_config.rx_queue_len = 32;
    g_config.tx_queue_len = 32;

  can_timing_config_t t_config = CAN_TIMING_CONFIG_250KBITS();
  can_filter_config_t f_config = CAN_FILTER_CONFIG_ACCEPT_ALL();

  if (can_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println("Failed to install driver");
    return;
  }

  if (can_start() != ESP_OK) {
    Serial.println("Failed to start driver");
    return;
  }
    
}

void ODriveTeensyCAN::sendMessage(int axis_id, int cmd_id, bool remote_transmission_request, int length, byte *signal_bytes) {
    can_message_t msg;
    can_message_t return_msg;

    msg.identifier = (axis_id << CommandIDLength) + cmd_id;
    msg.flags = remote_transmission_request;
    msg.data_length_code = length;
    if (!remote_transmission_request) {
        memcpy(msg.data, signal_bytes, sizeof(signal_bytes));
        can_transmit(&msg, pdMS_TO_TICKS(1000));
        return;
    }else{

    can_transmit(&msg, pdMS_TO_TICKS(1000));}
    while (true) {
        if ((can_receive(&return_msg, pdMS_TO_TICKS(1000)) == ESP_OK) && (return_msg.identifier == msg.identifier)) {
            memcpy(signal_bytes, return_msg.data, sizeof(return_msg.data));
            return;
        }
    }
}
void ODriveTeensyCAN::RequestUpdate(int axis_id, int cmd_id, uint32_t flag){
can_message_t msg;
  msg.identifier = (axis_id << CommandIDLength) + cmd_id;
    msg.flags = CAN_MSG_FLAG_RTR;
 can_transmit(&msg, pdMS_TO_TICKS(1000));
}

int ODriveTeensyCAN::Heartbeat() {
    can_message_t return_msg;
	if(can_receive(&return_msg, pdMS_TO_TICKS(1000)) == ESP_OK) {
		return (int)(return_msg.identifier >> 5);
	} else {
		return -1;
	}
}

void ODriveTeensyCAN::SetPosition(int axis_id, float position) {
    SetPosition(axis_id, position, 0.0f, 0.0f);
}

void ODriveTeensyCAN::SetPosition(int axis_id, float position, float velocity_feedforward) {
    SetPosition(axis_id, position, velocity_feedforward, 0.0f);
}

void ODriveTeensyCAN::SetPosition(int axis_id, float position, float velocity_feedforward, float current_feedforward) {
    int16_t vel_ff = (int16_t) (feedforwardFactor * velocity_feedforward);
    int16_t curr_ff = (int16_t) (feedforwardFactor * current_feedforward);

    byte* position_b = (byte*) &position;
    byte* velocity_feedforward_b = (byte*) &vel_ff;
    byte* current_feedforward_b = (byte*) &curr_ff;
    byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    msg_data[0] = position_b[0];
    msg_data[1] = position_b[1];
    msg_data[2] = position_b[2];
    msg_data[3] = position_b[3];
    msg_data[4] = velocity_feedforward_b[0];
    msg_data[5] = velocity_feedforward_b[1];
    msg_data[6] = current_feedforward_b[0];
    msg_data[7] = current_feedforward_b[1];

    sendMessage(axis_id, CMD_ID_SET_INPUT_POS, false, 8, position_b);
}

void ODriveTeensyCAN::SetVelocity(int axis_id, float velocity) {
    SetVelocity(axis_id, velocity, 0.0f);
}

void ODriveTeensyCAN::SetVelocity(int axis_id, float velocity, float current_feedforward) {
    byte* velocity_b = (byte*) &velocity;
    byte* current_feedforward_b = (byte*) &current_feedforward;
    byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    msg_data[0] = velocity_b[0];
    msg_data[1] = velocity_b[1];
    msg_data[2] = velocity_b[2];
    msg_data[3] = velocity_b[3];
    msg_data[4] = current_feedforward_b[0];
    msg_data[5] = current_feedforward_b[1];
    msg_data[6] = current_feedforward_b[2];
    msg_data[7] = current_feedforward_b[3];
    
    sendMessage(axis_id, CMD_ID_SET_INPUT_VEL, false, 8, velocity_b);
}

void ODriveTeensyCAN::SetVelocityLimit(int axis_id, float velocity_limit) {
    byte* velocity_limit_b = (byte*) &velocity_limit;

    sendMessage(axis_id, CMD_ID_SET_VELOCITY_LIMIT, false, 4, velocity_limit_b);
}

void ODriveTeensyCAN::SetTorque(int axis_id, float torque) {
    byte* torque_b = (byte*) &torque;

    sendMessage(axis_id, CMD_ID_SET_INPUT_TORQUE, false, 4, torque_b);
}

void ODriveTeensyCAN::ClearErrors(int axis_id) {
    sendMessage(axis_id, CMD_ID_CLEAR_ERRORS, false, 0, 0);
}

float ODriveTeensyCAN::GetPosition(int axis_id) {
    byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    sendMessage(axis_id, CMD_ID_GET_ENCODER_ESTIMATES, true, 8, msg_data);

    float_t output;
    *((uint8_t *)(&output) + 0) = msg_data[0];
    *((uint8_t *)(&output) + 1) = msg_data[1];
    *((uint8_t *)(&output) + 2) = msg_data[2];
    *((uint8_t *)(&output) + 3) = msg_data[3];

    return output;
}

float ODriveTeensyCAN::GetVelocity(int axis_id) {
    byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    sendMessage(axis_id, CMD_ID_GET_ENCODER_ESTIMATES, true, 8, msg_data);

    float_t output;
    *((uint8_t *)(&output) + 0) = msg_data[4];
    *((uint8_t *)(&output) + 1) = msg_data[5];
    *((uint8_t *)(&output) + 2) = msg_data[6];
    *((uint8_t *)(&output) + 3) = msg_data[7];
    return output;
}

uint32_t ODriveTeensyCAN::GetMotorError(int axis_id) {
    byte msg_data[4] = {0, 0, 0, 0};

    sendMessage(axis_id, CMD_ID_GET_MOTOR_ERROR, true, 4, msg_data);

    uint32_t output;
    *((uint8_t *)(&output) + 0) = msg_data[0];
    *((uint8_t *)(&output) + 1) = msg_data[1];
    *((uint8_t *)(&output) + 2) = msg_data[2];
    *((uint8_t *)(&output) + 3) = msg_data[3];
    return output;
}

uint32_t ODriveTeensyCAN::GetEncoderError(int axis_id) {
    byte msg_data[4] = {0, 0, 0, 0};

    sendMessage(axis_id, CMD_ID_GET_ENCODER_ERROR, true, 4, msg_data);

    uint32_t output;
    *((uint8_t *)(&output) + 0) = msg_data[0];
    *((uint8_t *)(&output) + 1) = msg_data[1];
    *((uint8_t *)(&output) + 2) = msg_data[2];
    *((uint8_t *)(&output) + 3) = msg_data[3];
    return output;
}

uint32_t ODriveTeensyCAN::GetAxisError(int axis_id) {
    byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint32_t output;

    can_message_t return_msg;

    int msg_id = (axis_id << CommandIDLength) + CMD_ID_ODRIVE_HEARTBEAT_MESSAGE;

    while (true) {
        if ((can_receive(&return_msg, pdMS_TO_TICKS(1000)) == ESP_OK) && (return_msg.identifier == msg_id)) {
            memcpy(msg_data, return_msg.data, sizeof(return_msg.data));
            *((uint8_t *)(&output) + 0) = msg_data[0];
            *((uint8_t *)(&output) + 1) = msg_data[1];
            *((uint8_t *)(&output) + 2) = msg_data[2];
            *((uint8_t *)(&output) + 3) = msg_data[3];
            return output;
        }
    }
}

uint32_t ODriveTeensyCAN::GetCurrentState(int axis_id) {
    byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint32_t output;

    can_message_t return_msg;

    int msg_id = (axis_id << CommandIDLength) + CMD_ID_ODRIVE_HEARTBEAT_MESSAGE;

    while (true) {
        if ((can_receive(&return_msg, pdMS_TO_TICKS(1000)) == ESP_OK) && (return_msg.identifier == msg_id)) {
            memcpy(msg_data, return_msg.data, sizeof(return_msg.data));
            *((uint8_t *)(&output) + 0) = msg_data[4];
            *((uint8_t *)(&output) + 1) = msg_data[5];
            *((uint8_t *)(&output) + 2) = msg_data[6];
            *((uint8_t *)(&output) + 3) = msg_data[7];
            return output;
        }
    }
}


float ODriveTeensyCAN::GetIQ(int axis_id) {
    byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    sendMessage(axis_id, CMD_ID_GET_IQ, true, 0, msg_data);


    //    Axis0IQ = *(myIq *)msg_data;
     float_t output;
    *((uint8_t *)(&output) + 0) = msg_data[4];
    *((uint8_t *)(&output) + 1) = msg_data[5];
    *((uint8_t *)(&output) + 2) = msg_data[6];
    *((uint8_t *)(&output) + 3) = msg_data[7];
    return output +1;
}

float ODriveTeensyCAN::GetVBus(int axis_id) {
    byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    sendMessage(axis_id, CMD_ID_GET_VBUS_VOLTAGE, true, 0, msg_data);


    //    Axis0IQ = *(myIq *)msg_data;
     float_t output;
    *((uint8_t *)(&output) + 0) = msg_data[0];
    *((uint8_t *)(&output) + 1) = msg_data[1];
    *((uint8_t *)(&output) + 2) = msg_data[2];
    *((uint8_t *)(&output) + 3) = msg_data[3];
    return output;
}

// void ODriveTeensyCAN::GetIQ(int axis_id) {

//     RequestUpdate(axis_id, CMD_ID_GET_IQ, true);

    
// }
// void ODriveTeensyCAN::update(){
    
//     can_message_t msg;
//     esp_err_t error;
//     while (esp_err_t (can_receive(&msg, 0) == ESP_OK)){
//         if (msg.flags) {
//             continue;
//         }
//          if (msg.data_length_code != 8) {
//             continue;
//         }
//         if(ODRIVE_AXIS_ID(0) == msg.identifier){
//             if(ODRIVE_CMD_ID(CMD_ID_GET_IQ) == msg.identifier ){
//                 Axis0IQ = *(myIq *)msg.data;
//                 break;
//             }else{
//                 break;
//             }
//         }
//     }
// }

bool ODriveTeensyCAN::RunState(int axis_id, int requested_state) {
    sendMessage(axis_id, CMD_ID_SET_AXIS_REQUESTED_STATE, false, 4, (byte*) &requested_state);
    return true;
}
