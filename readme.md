
# Esp32 Odrive Can


There is a modified version of firmware 5.4 for Odrive 3.6 56V in the lib to add the Function my_DATA and a Save_Configuration CAN.

There is also a workaround for changing between Input Modes/Control Modes

The Number of Motors should be set inside the LIB on NUMOFAXIS (default is 2)
The Esp32 need a **Can transceiver** TX and RX to CanH and CanL
This lib uses the esp can built-in driver (now is called TWAI for esp IDF, but this is running on Arduino)


Recieves **IQ_Measured, Control Mode, Input Mode, VBUS, Fet_Temperature** in one Can Msg.
The Command is 0x1C for myData and 0x1D for SaveConfiguration.

```
int16_t  vbus_voltage; //  float ratio -> 0.01
int16_t  Iq_measured; // float ratio -> 0.01
uint8_t  trajectory_done ; // bool
uint8_t  control_mode;
uint8_t  input_mode;
uint8_t  fet_temperature; //float ratio -> 0.5
```

**mydata under the OdriveFirmware:**

```bool  CANSimple::get_my_data_callback(const  Axis&  axis) { //ADD
can_Message_t  txmsg;
txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
txmsg.id += MSG_GET_MY_DATA; 
txmsg.isExt = axis.config_.can.is_extended;
txmsg.len = 8;

can_setSignal(txmsg, int16_t(vbus_voltage *100), 0, 16, true);
can_setSignal(txmsg, int16_t(axis.motor_.current_control_.Iq_measured_ *100), 16, 16, true);
can_setSignal(txmsg, uint8_t(axis.controller_.trajectory_done_), 32, 8, true);
can_setSignal(txmsg, uint8_t(axis.controller_.config_.control_mode), 40, 8, true);
can_setSignal(txmsg, uint8_t(axis.controller_.config_.input_mode), 48, 8, true);
can_setSignal(txmsg, uint8_t(axis.motor_.fet_thermistor_.temperature_ *2), 56, 8, true);

return  canbus_->send_message(txmsg);

}```