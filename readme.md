# Esp32 Odrive Can

There is a firmware in the lib to add the Function my_DATA and a Save_Configuration.
There is also a workaround for changing between Input Modes/Control Modes 

The Number of Motors should be set inside the LIB on NUMOFAXIS (default is 2)
The Esp32 need a **Can transceiver** TX and RX to CanH and CanL
This lib uses the esp can built-in driver (now is called TWAI for esp IDF, but this is running on Arduino)
