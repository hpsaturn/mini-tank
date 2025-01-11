#include <analogWrite.h>

bool inverted = false;

void led_clear() {
  #ifdef LED_INVERTED
  analogWrite(BUILTINLED, 255);
  #else
  analogWrite(BUILTINLED, 0);
  #endif
}

void led_write(int value) {
  #ifdef LED_INVERTED
    analogWrite(BUILTINLED, -value);
  #else
    analogWrite(BUILTINLED, value);
  #endif
}

void led_init() {
  pinMode(BUILTINLED, OUTPUT);
  led_clear();
}