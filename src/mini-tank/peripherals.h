#ifdef PERIPHERALS_ENABLE

bool camera_toggle = false;
bool led_lamp_toggle = false;

uint32_t trigger_led_lamp = 0;

void toggle_lamp(){
  if (trigger_led_lamp++ == 60) {
    led_lamp_toggle = !led_lamp_toggle;
    if (led_lamp_toggle) {
      digitalWrite(LED_LAMP_ENABLE, HIGH);
    }
    else {
      digitalWrite(LED_LAMP_ENABLE, LOW);
    }
    trigger_led_lamp = 0;
  }
}

uint32_t trigger_camera = 0;

void toggle_camera(){
  if (trigger_camera++ == 60) {
    camera_toggle = !camera_toggle;
    if (camera_toggle) {
      digitalWrite(CAMERA_ENABLE, HIGH);
    }
    else {
      digitalWrite(CAMERA_ENABLE, LOW);
    }
    trigger_camera = 0;
  }
}

void peripheralsInit(){
  pinMode(LED_LAMP_ENABLE, OUTPUT);
  digitalWrite(LED_LAMP_ENABLE, LOW);
  pinMode(CAMERA_ENABLE, OUTPUT);
  digitalWrite(CAMERA_ENABLE, LOW);
}
#else
void peripheralsInit(){}
void toggle_camera(){}
void toggle_lamp(){}
#endif