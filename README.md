# AIRobot

**! In development !**

ESP32 robot platform, the idea is join it with a [MaixPy RISC-V](https://maixpy.sipeed.com/en/) camera. For now the current development works with a M5StickC Joystick hat that controlling the robot via ESPNow using protobuf (nanopb). In the robot it use a simple ESP32 board. Also the old version use UDP, this version right now is in a branch.

## TODO

- [x] Platformio project (two sources)
- [x] UDP channel settings in preferences lib (old version)
- [x] nanopb (protocol buffers implementation) for joystick messages
- [x] separated OTA (joystick and robot) (old version)
- [x] ESPNow implementation isolated in a new library, [ESPNow Joystick](https://github.com/hpsaturn/espnow-joystick)
- [x] Added in joystick a main button action for "fire"
- [x] Added basic servo implementation with timers
- [x] Added telemetry feedback from robot to joystick
- [ ] OTA update over ESPNow version (master)
- [ ] SPI or I2C connection to MaxiPy nano camera
- [ ] Push AI models via proto
- [ ] Auto navegation
- [ ] Seek and destroy objects

## Firmware

You can build it with Arduino IDE renaming the main files to .ino, but it is more easy if you use PlatformIO, with a simple command you upload both, Joystick and Robot. Connect first the robot board and then the joystick to the USB of your computer and run:

```bash
pio run -e robot --target upload
pio run -e joystick --target upload
```

Please check the right USB ports on `platformio.ini` file.


## Usage

Turn on the robot, then the joystick, when the joystick detect the robot, push the M5 button for some seconds for pair, after that it should show the sticks values and controlling the robot. For turn off the joystick press again the M5 button.

## DIY Robot

For the instructions and more details [here](https://www.thingiverse.com/thing:4705776).

[![Youtube demo](http://img.youtube.com/vi/GmQLIsL-Mts/0.jpg)](http://www.youtube.com/watch?v=GmQLIsL-Mts "Joystick WiFi using nanopb (protobuff) over a ESP32 caterpillar")
