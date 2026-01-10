# Requirements
- NimBLE library tested with version 2.3.7
- ESP32
- 4 buttons
- 1 potentiometer
- 3D printed chasis
- cables, screws, heat inserts

# Known issues
- Sometimes ESP32 doesn't connect, replug/restart it
- Train after 1 minute doesn't connect, restart the train
- Speed control is disabled for now, you can change the code and second last byte in forward and backward packet encodes the speed
- water button is not actually water, it controls the purple action brick's action

Disclaimer: Large part of the code is AI generated, though I had to capture BLE communication, reverse enigneer it, test it, fix it, iterate and so on. For example the "cat" purple brick action is not available in the App, but is implemented here.