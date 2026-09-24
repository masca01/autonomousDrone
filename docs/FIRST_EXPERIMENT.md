# First ESP32 experiment

This test works with the ESP32 alone. An external LED is optional. It does not
require the IMU or control the built-in RGB LED.

1. Put the board on a nonconductive surface and connect the USB-C port labelled
   UART to your Mac with a data cable. Leave other hardware disconnected initially.
2. In VS Code open the Autonomous drone project folder.
3. Open PlatformIO's sidebar, expand Project Tasks > board_test > General, and
   select Upload. Select the serial port associated with your board if prompted.
4. After SUCCESS, choose Monitor under the same board_test environment.
5. At 115200 baud you should see alternating ON/OFF messages every half-second.
   These messages verify that your program is running even without an LED.

Select board_test explicitly; the default environment is the IMU program.
If no serial port appears, check the UART connector and the USB data cable.
Freenove includes CH343 driver and macOS troubleshooting resources here:
https://github.com/Freenove/Freenove_ESP32_S3_WROOM_Board

## Add a red LED

Unplug USB before wiring. Use a normal two-legged LED and a 330 ohm resistor:

```
GPIO 4 -> 330 ohm resistor -> LED long leg (+)
GND ----------------------> LED short leg (-)
```

GPIO 4 is the pin labelled 4 or IO4, not the fourth physical pin. On a typical
breadboard each numbered group of five holes is connected internally; the two
groups are separated by the centre gap. Put the LED legs in separate connected
groups. Do not connect both LED legs into the same group of five holes.

Reconnect USB. The external LED should turn on for 0.5 seconds and off for
0.5 seconds. The resistor must stay in series with the LED.

Change kBlinkIntervalMs in src/board_test.cpp to 1000 and upload again to make
each on/off period last one second. Close Monitor if it prevents uploading.
