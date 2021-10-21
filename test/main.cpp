
#include<Arduino.h>
#include "driver/gpio.h"
#include "driver/can.h"

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




void setup() {
  Serial.begin(115200);
  
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


void loop() {


 for (TickType_t tick = xTaskGetTickCount();; vTaskDelayUntil(&tick, 1)){

  can_message_t message;
  
  message.identifier =  ODRIVE_MSG_ID(0, ODRIVE_CMD_GET_ENCODER_ESTIMATES);;
  message.flags = CAN_MSG_FLAG_RTR ; // 
  message.data_length_code = 0; // ส่งข้อมูลยาว 6 ไบต์

  //  if (can_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
  //    Serial.println("Message queued for transmission");
  // } else {
  //   Serial.println("Failed to queue message for transmission");

  // }
  message.
  



  can_message_t msg;
if( can_receive(&msg, 0) == ESP_OK){
Serial.print("AXIS:");
Serial.print(ODRIVE_AXIS_ID(msg.identifier));
Serial.print("  CMD:");
Serial.print(ODRIVE_CMD_ID(msg.identifier));
Serial.println();
}else{
 Serial.println("NO MSG");

}
//debugcan();




//vTaskDelay(1);

}
}