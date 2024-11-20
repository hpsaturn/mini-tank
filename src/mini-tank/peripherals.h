#ifdef PERIPHERALS_ENABLE

int LED_LAMP_FRONT = 1;
int CAMERA_ENABLE = 3;

uint32_t trigger_led_lamp = 0;

void toggle_lamp(){
  if (trigger_led_lamp++ == 30) {
    led_lamp_toggle = !led_lamp_toggle;
    if (led_lamp_toggle) {
      digitalWrite(LED_LAMP_FRONT, HIGH);
    }
    else
      digitalWrite(LED_LAMP_FRONT, LOW);
    trigger_led_lamp = 0;
  }
}

uint32_t trigger_camera = 0;

void toggle_camera(){
  if (trigger_camera++ == 30) {
    camera_toggle = !camera_toggle;
    if (camera_toggle) {
      digitalWrite(CAMERA_ENABLE, HIGH);
    }
    else
      digitalWrite(CAMERA_ENABLE, LOW);
    trigger_camera = 0;
  }
}

void peripheralsInit(){
  pinMode(LED_LAMP_FRONT, OUTPUT);
  digitalWrite(LED_LAMP_FRONT, LOW);
  pinMode(CAMERA_ENABLE, OUTPUT);
  digitalWrite(CAMERA_ENABLE, LOW);
}
#else
void peripheralsInit(){}
void toggle_camera(){}
void toggle_lamp(){}
#endif