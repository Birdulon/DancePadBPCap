#include <USBComposite.h>
USBCompositeSerial CompositeSerial;
USBHID HID;
HIDJoystick Joystick(HID);

//const uint8_t reportDescription[] = {
//   //HID_MOUSE_REPORT_DESCRIPTOR(),
//   //HID_KEYBOARD_REPORT_DESCRIPTOR(),
//   HID_JOYSTICK_REPORT_DESCRIPTOR(),
//};

#define MODE INPUT_PULLDOWN
const uint8_t btn_pins[] = {PA0, PA1, PA2, PA3, PA4, PA5};
const uint8_t adc_gnd_pin = PB1;
const int NUM_BTNS = sizeof(btn_pins)/sizeof(btn_pins[0]);
const int us_per_loop = 2000;  // We want 500Hz polling
const int NUM_SAMPLES = 96;
const int NUM_SAMPLES_ELIMINATE = 32/2;
const int NUM_SAMPLES_SHIFT = 6;

uint32_t btn_values[NUM_BTNS] = {0};
bool btn_last_states[NUM_BTNS] = {0};
uint32_t btn_values_raw[NUM_SAMPLES][NUM_BTNS];
uint32_t btn_values_min[NUM_BTNS];
uint32_t btn_values_max[NUM_BTNS];

void setup() {
  pinMode(PB1, INPUT_PULLDOWN);
  pinMode(PB0, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  for (int i=0; i<NUM_BTNS; i++){
    pinMode(btn_pins[i], OUTPUT);
    digitalWrite(btn_pins[i], LOW);
  }
  calibrate();
  
  USBComposite.setManufacturerString("Luke's");
  USBComposite.setProductString("FlatMat");
  Joystick.setManualReportMode(true);
  
  HID.begin(HID_JOYSTICK);
  //HID.begin(CompositeSerial, reportDescription, sizeof(reportDescription));
}

void scan_pads(uint32_t output[NUM_BTNS]) {
  for (int i=0; i<NUM_BTNS; i++){
    pinMode(btn_pins[i], INPUT_PULLUP);
    //unsigned long start_pin_us = micros();  // may have to re-add for timing purposes
    analogRead(adc_gnd_pin);
    analogRead(adc_gnd_pin);  // discharge for twice as long for good measure
    output[i] = analogRead(btn_pins[i]);
    pinMode(btn_pins[i], OUTPUT);
    digitalWrite(btn_pins[i], LOW);
  }
}

void eliminate_outliers() {
  for (int pad=0; pad<NUM_BTNS; pad++){
    for (int pass=0; pass<NUM_SAMPLES_ELIMINATE; pass++){
      int s_min, s_max;
      int v_min = 0x7FFFFFFF;
      int v_max = 0;
      for (int s=0; s<NUM_SAMPLES; s++){
        int val = btn_values_raw[s][pad];
        if (val == 0)
          continue;
        if (val < v_min){
          v_min = val;
          s_min = s;
        }
        else if (val > v_max){
          v_max = val;
          s_max = s;
        }
      }
      btn_values_raw[s_min][pad] = 0;
      btn_values_raw[s_max][pad] = 0;
    }
  }
}

void full_scan() {
  for (int i=0; i<NUM_SAMPLES; i++)
    scan_pads(btn_values_raw[i]);
  eliminate_outliers();
}

void calibrate() {
  digitalWrite(LED_BUILTIN, HIGH);
  full_scan();
  for (int i=0; i<NUM_BTNS; i++) {
    btn_values_min[i] = 0x7FFFFFFF; //btn_values_raw[0][i];
    btn_values_max[i] = 0;  //btn_values_raw[0][i];
    for (int j=0; j<NUM_SAMPLES; j++) {
      if (btn_values_raw[j][i] > btn_values_max[i])
        btn_values_max[i] = btn_values_raw[j][i];
      if ((btn_values_raw[j][i] != 0) and (btn_values_raw[j][i] < btn_values_min[i]))
        btn_values_min[i] = btn_values_raw[j][i];
    }
  }
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  unsigned long start_us = micros();
  if (!digitalRead(PB0))
    calibrate();
  full_scan();
  for (int i=0; i<NUM_BTNS; i++) {
    btn_values[i] = btn_values_raw[0][i];
    for (int j=1; j<NUM_SAMPLES; j++)
      btn_values[i] += btn_values_raw[j][i];
    btn_values[i] >>= NUM_SAMPLES_SHIFT;
    float multiplier = btn_last_states[i] ? 0.999999 : 0.996;
    bool new_state = btn_values[i] < btn_values_min[i]*multiplier;
    Joystick.button(i+1, new_state);
    btn_last_states[i] = new_state;
  }
//  Joystick.X(btn_values[0]>>2);
//  Joystick.Y(btn_values[1]>>2);
//  Joystick.Xrotate(btn_values[2]>>2);
//  Joystick.Yrotate(btn_values[3]>>2);
//  Joystick.sliderLeft(btn_values[4]>>2);
//  Joystick.sliderRight(btn_values[5]>>2);
  Joystick.send();
//  char str[130];
//  sprintf(str, "Buttons: %d  %d  %d  %d  %d  %d", btn_values[0], btn_values[1], btn_values[2], btn_values[3], btn_values[4], btn_values[5]);
//  CompositeSerial.println(str);
  while ((unsigned long)(micros() - start_us) < us_per_loop) {};  // We want consistent loop timing
}
