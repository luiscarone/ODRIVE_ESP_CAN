#include <arduino.h>

#include "odrive_can.h"
#include "driver/can.h"
#include "freertos/task.h"
#include <esp_log.h>
#include <esp_err.h>

#define PIN_CAN_TX GPIO_NUM_13
#define PIN_CAN_RX GPIO_NUM_12
#define CAN_BAUD_RATE can_TIMING_CONFIG_250KBITS
#define N_MOTORS 2



static void odrive_state_transition_callback(
    uint8_t axis_id, ODriveAxisState new_state, ODriveAxisState old_state, void *context)
{
  printf("axis_id %u, new_state %u, old_state %u\n", axis_id, new_state, old_state);
}

ODriveAxis axes[N_MOTORS];

ODriveControllerModes mode = {
    ODRIVE_CONTROL_MODE_POSITION,
    ODRIVE_INPUT_MODE_PASSTHROUGH,
};

void setup()
{
can_general_config_t g_config = { // สร้างต้วแปร g_config ใช้กำหนดค่าเกี่ยวกับบัส CAN
    .mode = CAN_MODE_NORMAL,
    .tx_io = GPIO_NUM_12, // กำหนดขา TX ต่อกับ 26
    .rx_io = GPIO_NUM_13, // กำหนดขา TX ต่อกับ 27
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
}

can_message_t msg;
can_message_t rcvmsg;
void loop()
{

  // if (can_receive(&rcvmsg, 100) == ESP_OK)
  // {
  //   Serial.print("YEP");
  // }
  // else
  // {
  //   Serial.print("NONE");
  // }
}
