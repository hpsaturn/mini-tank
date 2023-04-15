#include <ESP32Servo.h>
#include <U8g2lib.h>
#include <analogWrite.h>

#include <EspNowJoystick.hpp>

#include "GUIIcons.h"

#define BUILTINLED 22

EspNowJoystick joystick;
TelemetryMessage tm;

Servo servoLeft;
Servo servoRight;
int servoLeftPin = 16;
int servoRightPin = 17;

bool running, fire;
uint32_t count = 0;

U8G2_SSD1306_64X48_ER_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, U8X8_PIN_NONE, U8X8_PIN_NONE);
int dw = 0;  // display width
int dh = 0;  // display height

int lastDrawedLine = 0;

const int spanLeft = 30;
const int offsetLeft = 0;
const int degreesCenterL = 96;
const int degreesMinL = degreesCenterL - spanLeft + offsetLeft;
const int degreesMaxL = degreesCenterL + spanLeft + offsetLeft;

const int spanRight = 30;
const int offsetRight = -2;
const int degreesCenterR = 99;
const int degreesMinR = degreesCenterR - spanRight + offsetRight;
const int degreesMaxR = degreesCenterR + spanRight + offsetRight;

int lastVty = 0;

void showWelcome() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(0, 0, "MiniTank v001");
  u8g2.sendBuffer();
  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(dw - 18, 1, String(SRC_REV).c_str());
  u8g2.drawLine(0, 9, dw - 1, 9);
  lastDrawedLine = 10;
  // only for first screen
  u8g2.sendBuffer();
}

void showWelcomeMessage(String msg) {
  if (lastDrawedLine >= dh - 6) {
    delay(500);
    showWelcome();
  }
  u8g2.setFont(u8g2_font_4x6_tf);
  if (dh == 32) {
    if (lastDrawedLine < 32) {
      u8g2.drawStr(0, lastDrawedLine, msg.c_str());
    } else {
      u8g2.drawStr(72, lastDrawedLine - 20, msg.c_str());
    }
  } else
    u8g2.drawStr(0, lastDrawedLine, msg.c_str());
  lastDrawedLine = lastDrawedLine + 7;
  u8g2.sendBuffer();
}

void displayEmoticonLabel(int cursor, String msg) {
  u8g2.setFont(u8g2_font_unifont_t_emoticons);
  u8g2.drawGlyph(76, 12, cursor);
  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.setCursor(77, 17);
  u8g2.print(msg);
}

void displayEmoticon() {
  static uint_least32_t smileTStamp = 0;
  if (millis() - smileTStamp > 500) {
    smileTStamp = millis();
    if(lastVty == 0) return;
    int speed = lastVty;
    u8g2.clearBuffer();
    u8g2.sendBuffer();
    if (speed > 60)
      u8g2.drawXBM(15, 2, 32, 32, SmileFaceGood);
    else if (speed > 0)
      u8g2.drawXBM(15, 2, 32, 32, SmileFaceModerate);
    else if (speed < -70)
      u8g2.drawXBM(15, 2, 32, 32, SmileFaceHazardous);
    else if (speed < -50)
      u8g2.drawXBM(15, 2, 32, 32, SmileFaceVeryUnhealthy);
    else if (speed < -40)
      u8g2.drawXBM(15, 2, 32, 32, SmileFaceUnhealthySGroups);
    else if (speed < -20)
      u8g2.drawXBM(15, 2, 32, 32, SmileFaceUnhealthy);
    else
      u8g2.drawXBM(15, 2, 32, 32, SmileFaceModerate);
    u8g2.sendBuffer();
  }
}

void displayBigMsg(String msg) {
  u8g2.setFont(u8g2_font_inb19_mn);
  int strw = u8g2.getStrWidth(msg.c_str());
  u8g2.setCursor((dw - strw) / 2, 1);
  u8g2.print(msg.c_str());
}

void displayBottomLine(String msg) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_5x7_tf);
  int strw = u8g2.getStrWidth(msg.c_str());
  u8g2.setCursor((dw - strw) / 2, 25);
  u8g2.print(msg.c_str());
  u8g2.sendBuffer();
}

void displayInit() {
  u8g2.setBusClock(100000);
  u8g2.begin();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setContrast(128);
  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);
  u8g2.setFontMode(0);
  dw = u8g2.getDisplayWidth();
  dh = u8g2.getDisplayHeight();
  Serial.println("-->[OGUI] display config ready.");
}

void attachServoLeft() {
  if (!servoLeft.attached()) servoLeft.attach(servoLeftPin);
}

void attachServoRight() {
  if (!servoRight.attached()) servoRight.attach(servoRightPin);
}

void detachServos() {
  servoLeft.detach();
  servoRight.detach();
}

void setSpeed(int16_t Vtx, int16_t Vty, int16_t Wt) {
  Vtx = constrain(-Wt, -100, 100);
  Vty = constrain(Vty, -100, 100);
  //   Wt = constrain(Wt, -100, 100);

  const int deathBand = 3;
  int spdL;
  int spdR;

  if (abs(Vtx) < deathBand && abs(Vty) < deathBand) {
    Vtx = 0;
    Vty = 0;
  }
 
  // Mixer
  spdL = Vty + Vtx;   //motorDelanteroIzquierdo
  spdR = -Vty + Vtx;  //motorDelanteroDerecho

  // Servo output
  spdL = map(spdL, -100, 100, degreesMinL, degreesMaxL);
  spdR = map(spdR, -100, 100, degreesMinR, degreesMaxR);

  //   Serial.printf("[spdR:%04d spdL:%04d]\r\n", spdR, spdL);

  if (spdL != degreesCenterL) {
    attachServoLeft();
    servoLeft.write(spdL);
  } else {
    servoLeft.detach();
  }

  if (spdR != degreesCenterR) {
    attachServoRight();
    servoRight.write(spdR);
  } else {
    servoRight.detach();
  }
  // GUI Variables
  if(lastVty!=0) lastVty = Vty; 
  if(Vty!=0) lastVty = Vty;
  analogWrite(BUILTINLED, abs(Vty));
}

void sendHeartbeat() {
  static uint_least32_t timeStamp = 0;
  if (millis() - timeStamp > 500) {
    timeStamp = millis();
    tm.e1 = true;
    joystick.sendTelemetryMsg(tm);
  }
}

void checkFire() {
  if (fire) {
    Serial.println("Fire");
    // servo1.write(53);
    fire = false;
  }
  // else
  // servo1.write(70);
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
  joystick.init();
  showWelcomeMessage("ESPNow ready");

  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  attachServoLeft();
  attachServoRight();
  showWelcomeMessage("Servos ready");
  delay(500);
  showWelcomeMessage("== SETUP READY ==");
}

void loop() {
  checkFire();
  checkRunning();
  sendHeartbeat();
  displayEmoticon();
  delay(20);
}
