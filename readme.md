# Requirements
- NimBLE library tested with version 2.5.1
- ESP32 (tested with ESP32 Arduino core 3.3.12)
- 4 buttons
- 1 potentiometer
- 3D printed chasis
- cables, screws, heat inserts
- optional: Xbox Wireless Controller (Series X|S, Elite 2 or One S model 1708) with up-to-date firmware

# Xbox controller
The ESP32 connects to the train and an Xbox controller at the same time over Bluetooth LE. The physical buttons keep working.

First pairing: turn on the train, then hold the controller's pair button until the Xbox logo blinks fast. The controller is remembered, so after that just press the Xbox button. Hold the Stop button while the ESP32 boots to forget the controller.

| Control | Action |
|---|---|
| Left / right stick up, down | Forward / backward, speed follows the stick, release stops |
| RT / LT | Forward / backward, speed follows the trigger, release stops |
| D-pad up / down | Forward / backward at default speed, keeps running |
| A | Horn |
| B | Stop |
| X / LB | Next / previous light color |
| Y / RB | Next / previous action |

Set `kXboxDebug` to `true` to print controller discovery and raw input reports to Serial.

# Known issues
- Sometimes ESP32 doesn't connect, replug/restart it
- Train after 1 minute doesn't connect, restart the train
- Weak USB power can trigger the brownout detector and reboot the ESP32, use a good cable/power supply
- The ESP32 keeps the same BLE address across reboots (`kStableAddress`) so the controller reconnects; if the train stops connecting, set it to `false` and pair the controller on every boot
- water button is not actually water, it controls the purple action brick's action

Disclaimer: Large part of the code is AI generated, though I had to capture BLE communication, reverse enigneer it, test it, fix it, iterate and so on. For example the "cat" purple brick action is not available in the App, but is implemented here.
