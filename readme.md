# Requirements
- NimBLE library tested with version 2.3.7
- ESP32
- 4 buttons
- 1 potentiometer
- 3D printed chasis
- cables, screws, heat inserts

# Xbox controller version
Want to drive the train with an Xbox controller? Use the [`xbox-controller` branch](https://github.com/drndos/duplo-train-controller/tree/xbox-controller). The ESP32 connects to the train and an Xbox Wireless Controller (Series X|S, Elite 2 or One S model 1708, with up-to-date firmware) at the same time over Bluetooth LE. The physical buttons keep working.

- Sticks and triggers drive forward/backward with variable speed, release stops
- D-pad up/down drives at the default speed, A horn, B stop, X/LB lights, Y/RB actions
- The controller is remembered, so it reconnects with the Xbox button after the first pairing

Pairing steps, the full control table and extra known issues are in that branch's readme. It was tested with NimBLE 2.5.1 and ESP32 Arduino core 3.3.12.

# Known issues
- Sometimes ESP32 doesn't connect, replug/restart it
- Train after 1 minute doesn't connect, restart the train
- Speed control is disabled for now, you can change the code and second last byte in forward and backward packet encodes the speed
- water button is not actually water, it controls the purple action brick's action

Disclaimer: Large part of the code is AI generated, though I had to capture BLE communication, reverse enigneer it, test it, fix it, iterate and so on. For example the "cat" purple brick action is not available in the App, but is implemented here.