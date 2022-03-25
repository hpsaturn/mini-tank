#include <EspNowJoystick.hpp>
#include <analogWrite.h>
#include <ESP32Servo.h>

#define BUILTINLED  22

EspNowJoystick joystick;
TelemetryMessage tm;
Servo servoLeft;
Servo servoRight;
int servoLeftPin = 2;
int servoRightPin = 4;

bool running,fire;
uint32_t count = 0;


#define SERVO_STOP 101
#define SERVO_MAX  116


void setSpeed(int16_t Vtx, int16_t Vty, int16_t Wt) {
    Wt = (Wt > 100) ? 100 : Wt;
    Wt = (Wt < -100) ? -100 : Wt;

    Vtx = (Vtx > 100) ? 100 : Vtx;
    Vtx = (Vtx < -100) ? -100 : Vtx;

    Vty = (Vty > 100) ? 100 : Vty;
    Vty = (Vty < -100) ? -100 : Vty;

    int speedR = map(abs(Vty), 0, 100, SERVO_STOP, SERVO_MAX);
    int speedL = SERVO_STOP-(speedR-SERVO_STOP);

    
    int turnR = map(abs(Wt), 0, 100, 0, SERVO_MAX-SERVO_STOP);
    int turnL = turnR;

    if (Wt>0) turnR = 0;
    if (Wt<0) turnL = 0;
    if (Wt==0) turnR = turnL = 0;
    


    if (Vty > 2) {
        // Serial.printf("[Vtx:%04d Vty:%04d Wt:%04d]\n", Vtx, Vty, Wt);
        Serial.printf("[spdR:%04d spdL:%04d turnR:%04d turnL:%04d]\n", speedR, speedL, turnR, turnL);
        servoLeft.write(speedR + turnR);
        servoRight.write(speedL - turnL);
        analogWrite(BUILTINLED, abs(Vty));
    } else if (Vty < -2) {
        servoLeft.write(speedL - turnL);
        servoRight.write(speedR + turnR);
        analogWrite(BUILTINLED, abs(Vty));
    }
    else if (abs(Vty) <= 2 && abs(Vty) >=0 && Wt !=0 ) {
        if(Wt>0) {
            servoLeft.write(SERVO_STOP-turnL);
            servoRight.write(SERVO_STOP-turnL);
        }
        else {
            servoRight.write(SERVO_STOP+turnR);
            servoLeft.write(SERVO_STOP+turnR);
        }
        analogWrite(BUILTINLED, abs(Wt));
    }
    else {
        servoLeft.write(SERVO_STOP);
        servoRight.write(SERVO_STOP);
        analogWrite(BUILTINLED, 0);
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
    void onJoystickMsg(JoystickMessage jm){
        // Serial.println("[Joystick]");
        connectStamp = millis();
        if (jm.ck == 0x02 && jm.bA == 1) {
            fire = true;
        }
        if (jm.ck == 0x01) { 
            setSpeed(jm.ax - 100, jm.ay - 100, jm.az - 100); 
            running = true;
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

    joystick.setJoystickCallbacks(new MyJoystickCallback());
    tm = joystick.newTelemetryMsg();
    joystick.init();

    // Allow allocation of all timers
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    servoLeft.setPeriodHertz(50);  // Standard 50hz servo
    servoLeft.attach(servoLeftPin,500,2400);
    servoRight.setPeriodHertz(300);  // Standard 50hz servo
    servoRight.attach(servoRightPin,500,2400);
}

void loop() {
    checkFire(); 
    checkRunning();
    sendHeartbeat();
    delay(5);
}
