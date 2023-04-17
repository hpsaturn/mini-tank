
## Mini Tank

ESP32/8266 robot platform using two basic servos.

## Credits

- 3D Print design and original idea from [Francisco Carabaza @acicuecalo](https://sites.google.com/view/robot-mini-tanque/inicio)
- Icons and graphics from [CanAirIO Project](https://github.com/kike-canaries/canairio_firmware#readme)
- Original firmware [AIRobot](https://github.com/hpsaturn/airobot#readme)

## TODO

- [x] Two Joystick flavors (M5StickC and M5StickCPlus)
- [x] Basic emoticons handling
- [x] ESPNow implementation using the [ESPNow Joystick](https://github.com/hpsaturn/espnow-joystick) library
- [x] Servo control improvements. Thanks to [@acicuecalo](https://github.com/acicuecalo)
- [ ] OTA update in boot
- [ ] Battery level
- [ ] SPI or I2C connection to MaxiPy nano camera ?

## Wiring

The robot in the video has the next alternartive componentes:

- [ ] TTGO T7 ESP32 Board (is possible Wemos ESP8266, see below)
- [ ] D1 Mini proto PCB (for wire connections and connectors)
- [ ] D1 Mini OLED shield (optional)
- [ ] Two small servos (SG90 or compatible modified to 360¤)
- [ ] LIPO battery of 3.7v
- [ ] Two M3/M2 screws (also with servo screw maybe works)
- [ ] Two Rubber O-Rings Seal 35mm OD 31mm ID 2mm Width

The pins used to connect the servos are: 16,17.

## Firmware

You can build and upload it using PlatformIO, with a simple command you upload both, Joysticks and Robot. Connect first the robot board and then the joystick to the USB of your computer and run:

```bash
pio run -e robot --target upload
pio run -e joystickM5StickC --target upload
pio run -e joystickM5StickCPlus --target upload
```

## 3D Printing

For 3DPrint files, build instructions and components details [here](https://sites.google.com/view/robot-mini-tanque/inicio)

[![Youtube demo](images/youtube_preview.jpg)](https://youtu.be/I6cGg1o1NR0 "Joystick WiFi using nanopb (protobuff) over a ESP32 caterpillar")
