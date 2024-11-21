#include <EspNowJoystick.hpp>
#include <ESP32Servo.h>
#include <analogWrite.h>
#include "GUI.h"
#include "peripherals.h"

EspNowJoystick joystick;
TelemetryMessage tm;

Servo servoLeft;
Servo servoRight;
int servoLeftPin = SERVO_LEFT_PIN;
int servoRightPin = SERVO_RIGHT_PIN;

const int spanLeft = SPAN_LEFT;
const int offsetMinLeft = OFFSET_MIN_LEFT;
const int offsetMaxLeft = OFFSET_MAX_LEFT;
const int degreesCenterL = CENTER_LEFT;
#ifndef SERVO_INVERTED
const int degreesMinL = degreesCenterL + spanLeft;
const int degreesMaxL = degreesCenterL - spanLeft;
#else
const int degreesMinL = degreesCenterL - spanLeft;
const int degreesMaxL = degreesCenterL + spanLeft;
#endif

const int deathBand = DEATH_BAND;

const int spanRight = SPAN_RIGHT;
const int offsetMinRight = OFFSET_MIN_RIGHT;
const int offsetMaxRight = OFFSET_MAX_RIGHT;
const int degreesCenterR = CENTER_RIGHT;
#ifndef SERVO_INVERTED
const int degreesMinR = degreesCenterR + spanRight;
const int degreesMaxR = degreesCenterR - spanRight;
#else
const int degreesMinR = degreesCenterR - spanRight;
const int degreesMaxR = degreesCenterR + spanRight;
#endif

bool running, fire;
uint32_t count = 0;
int lastVty = 0;

void attachServoLeft() {
  if (!servoLeft.attached()) {
    servoLeft.attach(servoLeftPin);
  }
}

void attachServoRight() {
  if (!servoRight.attached()) {
    servoRight.attach(servoRightPin);
  } 
}

void detachServos() {
  servoLeft.detach();
  servoRight.detach();
}

void servo_brake(Servo servo) {
  #ifdef SERVO_BRAKE
    for (int i = 0; i < SERVO_BRAKE; i++) servo.write(degreesCenterR + deathBand*3);
    for (int i = 0; i < SERVO_BRAKE; i++) servo.write(degreesCenterR - deathBand*3);
  #endif
}

/**
 * @param Vtx Joystick left stick, X axis (Left/Right)
 * @param Vty Joystick left stick, Y axis (forward/backward)
 * @param Wt Joystick right stick, X asis (Left/Right) 
 * 
 * The next code to controlling the servos. You able to capture more data from the Joystick Object,
 * for now, for simplify, we only use two sticks but only two axis.
 * 
 * The current version was improved from
 * the code @acicuecalo for Arduino IDE:
 * https://github.com/acicuecalo/Robot_mini_tanque
*/
void setSpeed(int16_t ax, int16_t Vty, int16_t Wt) {
  int Vtx = constrain(-Wt, -100, 100);
  Vty = constrain(Vty, -100, 100);
  ax = constrain(ax, -100, 100);

  int spdL;
  int spdR;

  if (abs(Vtx) < deathBand && abs(Vty) < deathBand) {
    Vtx = 0;
    Vty = 0;
  }
 
  // Mixer
  spdL = Vty + Vtx;   //motorL
  spdR = -Vty + Vtx;  //motorR

  // Servo output
  spdL = map(spdL, -100, 100, degreesMinL, degreesMaxL);
  spdR = map(spdR, -100, 100, degreesMinR, degreesMaxR);
   
  if (spdL != degreesCenterL && abs(spdL) > deathBand) {
    attachServoLeft();
    if (spdL > degreesCenterL) spdL = spdL + offsetMaxLeft;
    else spdL = spdL - offsetMinLeft;
    servoLeft.write(spdL);
  } else if (servoLeft.attached()) {
    servo_brake(servoLeft);
    servoLeft.detach();
  }

  if (spdR != degreesCenterR && abs(spdR) > deathBand) {
    attachServoRight();
    if (spdR > degreesCenterR) spdR = spdR + offsetMaxRight;
    else spdR = spdR - offsetMinRight;
    // Serial.printf("[spdR:%04d]\r\n", spdR);
    servoRight.write(spdR);
  } else if (servoRight.attached()) {
    servo_brake(servoRight);
    servoRight.detach();
  }

  if (ax < -90 && spdL > 50) toggle_lamp();
  if (ax > 87 && spdL < 45) toggle_camera();

  // GUI Variables
  if(lastVty!=0) lastVty = Vty; 
  if(Vty!=0) lastVty = Vty;

  // Debugging
  if (spdL !=degreesCenterL || spdR != degreesCenterR) {
    Serial.printf("[spdR:%04d spdL:%04d ax:%04d]\r\n", spdR, spdL, ax);
    analogWrite(BUILTINLED, (int)map(spdL, degreesMinL, degreesMaxL, 0, 254));
  }
}

void sendHeartbeat() {
  static uint_least32_t timeStamp = 0;
  if (millis() - timeStamp > 500) {
    timeStamp = millis();
    tm.e1 = true;
    joystick.sendTelemetryMsg(tm);
  }
}

static uint_least32_t connectStamp = 0;

void checkRunning() {
  if (millis() - connectStamp > 100) {
    running = false;
    setSpeed(0, 0, 0);
  }
}

class MyJoystickCallback : public EspNowJoystickCallbacks {
  void onJoystickMsg(JoystickMessage jm) {
    // Serial.println("[Joystick]");
    connectStamp = millis();
    if (jm.ck == 0x02 && jm.bA == 1) {
      fire = true;
    }
    if (jm.ck == 0x01) {
      static uint_least32_t speedStamp = 0;
      if (millis() - speedStamp > 20) {
        speedStamp = millis();
        setSpeed(jm.ax - 100, jm.ay - 100, jm.az - 100);
        running = true;
      }
    } 
  };
  void onError(const char* msg) {
    setSpeed(0, 0, 0);
    Serial.println("Error");
  };
};

void setup() {
  Serial.begin(115200);
  delay(100);
  displayInit();
  showWelcome();
  joystick.setJoystickCallbacks(new MyJoystickCallback());
  tm = joystick.newTelemetryMsg();
  joystick.devmode = true;
  joystick.init();
  showWelcomeMessage("ESPNow ready");

  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  attachServoLeft();
  attachServoRight();
  peripheralsInit();
  showWelcomeMessage("Servos ready");
  delay(500);
  showWelcomeMessage("== SETUP READY ==");
}

void loop() {
  checkRunning();
  sendHeartbeat();
  displayEmoticon(lastVty);
  delay(20);
}
