
# b-l475e-iot01a-ble-sensor-example

BLE sensor example using the St Micro BL-475E-IOT01A Discovery IoT board

This is a simple BLE sensor demo that does the following:

- uses a ST Microelectronics BL-475E-IOT01A Discovery Kit running an Arduino sketch and acting as a temperature/humidity sensor to:
-- advertise environmental and time services
-- upon successful establishment of BLE connection print via console every second temperature and humdity values from the on-board HTS221 sensor
-- transmit values via BLE to connected central device
-- transmit a time notification every minute when notification enabled by connected central device


Corresponding Python script developed for a Raspberry Pi 3 to act as the BLE central device can be found in the following repository:

https://github.com/msbaylis/rpi3-aws-iot-ble-sensor-example.git


