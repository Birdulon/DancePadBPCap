#include <USBComposite.h>
USBCompositeSerial CompositeSerial;
USBHID HID;
HIDJoystick Joystick(HID);

// Change these strings to match the mat you're flashing it to
#define MANUFACTURER "LHW"
//#define PRODUCT "FlatMat" //"FatMat"
#define PRODUCT "FatMat2"

//const uint8_t reportDescription[] = {
//   //HID_MOUSE_REPORT_DESCRIPTOR(),
//   //HID_KEYBOARD_REPORT_DESCRIPTOR(), 
//   HID_JOYSTICK_REPORT_DESCRIPTOR(),
//};

// If you wish to disable the axes reporting (e.g. to prevent it interfering with bad software), comment this define out.
#define JOYSTICK_AXES

// If you mess up your physical connections you can change them here :)
const uint8_t btn_pins[] = {PA0, PA1, PA2, PA3, PA4, PA5};
const uint8_t adc_gnd_pin = PB1;

const uint8_t recal_switch_pin = PB6;  //PB0;
const int NUM_BTNS = sizeof(btn_pins)/sizeof(btn_pins[0]);
const int us_per_loop = 2000;  // We want 500Hz polling. Change to 1000 for 1kHz polling, though you might need to reduce samples.
const int NUM_SAMPLES = 100;  // Sample each pad this many times, each polling cycle. Higher numbers should be less susceptible to noise, but may require slower polling rates.
const int NUM_SAMPLES_ELIMINATE = 36/2;  // The elimination routine chops the max and min value each pass, hence the factor of 2.
const int NUM_SAMPLES_SHIFT = 6;  // Leftshift the sum of all the remaining samples by this many bits for the result.

const int VALUE_BUFFER_SIZE = 4;
int value_index = 0;
int btn_values[VALUE_BUFFER_SIZE][NUM_BTNS] = {0};
bool btn_last_states[NUM_BTNS] = {0};
uint32_t btn_values_raw[NUM_SAMPLES][NUM_BTNS];
int btn_values_max[NUM_BTNS];
int btn_values_min[NUM_BTNS];
int btn_values_range[NUM_BTNS];
int btn_values_cal[NUM_BTNS];
bool last_calibrating = false;

void setup() {
  // We need to be able to discharge the ADC capacitor between pad reads to have any hope of consistency.
  // In the past, I played around with setting a channel as an output but it seemed dubious so I use a physical wire to achieve proper grounding of the channel.
  pinMode(adc_gnd_pin, INPUT_PULLDOWN);
  // pinMode(adc_gnd_pin, OUTPUT);
  // digitalWrite(adc_gnd_pin, LOW);

  pinMode(recal_switch_pin, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  for (int btn=0; btn<NUM_BTNS; btn++){
    pinMode(btn_pins[btn], OUTPUT);
    digitalWrite(btn_pins[btn], LOW);
  }
  // Grounding pin for Recal switch
  pinMode(PB7, OUTPUT);
  digitalWrite(PB7, LOW);
  //Set sampling clock cycles for the ADC: ADC_SMPR_7_5 ADC_SMPR_28_5 ADC_SMPR_41_5
  ADC1_BASE->SMPR2 = ADC_SMPR_7_5 <<(9*3);
  calibrate(true);
  
  USBComposite.setManufacturerString(MANUFACTURER);
  USBComposite.setProductString(PRODUCT);
  Joystick.setManualReportMode(true);
  
  HID.begin(HID_JOYSTICK);
  //HID.begin(CompositeSerial, reportDescription, sizeof(reportDescription));
}

void scan_pads(uint32_t output[NUM_BTNS]) {
  /* Scan each pad in order, once.
  The scan method is a crude RC measurement - 
  the pad is charged for a set amount of time by a voltage source with series resistance,
  and the voltage at the end of that time is measured.
  Higher capacitance values will result in lower voltages at the time of measurement,
  and conversely lower capacitance values will result in higher voltages.
  Larger pads will exhibit more base capacitance than smaller ones,
  which you'll see when comparing the Start/Back pads to the four directional pads.
  In general, human feet hovering above the pads will increase the apparent capacitance,
  and if the human body is grounded or has a stronger capacitive coupling to ground,
  (e.g. by having another foot over a grounded sensor pad) the increase will be even greater.
  In theory, you could attach an anti-static wristband to your ankle and link it to ground
  for maximum effect, but that would be very uncomfortable to play with. */
  for (int btn=0; btn<NUM_BTNS; btn++){
    // This begins to charge the sensor pad using the 30~50 kohm internal pullup resistor.
    pinMode(btn_pins[btn], INPUT_PULLUP);
    // We make the ADC sample GND to discharge the internal capacitor, and ignore the result.
    // For debugging purposes you could check the value if you want to confirm it's close enough to zero.
    analogRead(adc_gnd_pin);
    // Now we read the voltage that has accumulated on the sensor pad in the meantime.
    // Note that the capacitance of the ADC (and thus the charge siphoned from the pad) is significant in this arrangement.
    // This is the reason that we need the ADC's capacitor to be discharged to get a good reading.
    output[btn] = analogRead(btn_pins[btn]);
    // Now we discharge the sensor pad with a low impedence 0V output and leave it there, so it's ready for next time.
    pinMode(btn_pins[btn], OUTPUT);
    digitalWrite(btn_pins[btn], LOW);
  }
}

void eliminate_outliers() {
  // Chop the highest and the lowest values for each pad in an unsorted, constant time fashion.
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
  // Do a series of scans and remove the outliers.
  for (int sample=0; sample<NUM_SAMPLES; sample++)
    scan_pads(btn_values_raw[sample]);
  eliminate_outliers();
}

void calibrate(bool reset) {
  // Treat the current pad values as steady-state.
  digitalWrite(LED_BUILTIN, HIGH);
  full_scan();
  for (int btn=0; btn<NUM_BTNS; btn++) {
    if (reset){
      btn_values_min[btn] = 0x7FFFFFFF; //btn_values_raw[0][btn];
      btn_values_max[btn] = 0;  //btn_values_raw[0][btn];
    }
    for (int j=0; j<NUM_SAMPLES; j++) {
      if (btn_values_raw[j][btn] > btn_values_max[btn])
        btn_values_max[btn] = btn_values_raw[j][btn];
      if ((btn_values_raw[j][btn] != 0) and (btn_values_raw[j][btn] < btn_values_min[btn]))
        btn_values_min[btn] = btn_values_raw[j][btn];
    }
    btn_values_range[btn] = (btn_values_max[btn] - btn_values_min[btn]);
  }
  digitalWrite(LED_BUILTIN, LOW);
}

bool evaluate_button_state(int btn){
  // Determine if a pad is pressed or not.
  // I tend to have different tweak factors for different mat designs, you might need to mess around a bit to get behaviour you like.
  int comparison;
  //#define ABS_COMPARISON
  #define ABS_COMP_ON  -72 //-88   //-22 on joystick val
  #define ABS_COMP_OFF  -70 //-80  //-20 on joystick val

  #ifdef ABS_COMP_ON
    if (btn_last_states[btn])
      return (btn_values[value_index][btn] < btn_values_cal[btn] + ABS_COMP_OFF);
    else
      return (btn_values[value_index][btn] < btn_values_cal[btn] + ABS_COMP_ON);
  #else
    if (btn_last_states[btn]){
      // For a release, the latest samples must all be higher than the last reference comparison
      #ifdef ABS_COMPARISON
        comparison = btn_values_min[btn] - (btn_values_range[btn]>>6);
      #else
        comparison = btn_values[(value_index+1) % VALUE_BUFFER_SIZE][btn] + btn_values_range[btn];
      #endif
      for (int j=0; j<VALUE_BUFFER_SIZE-1; j++){
        if (btn_values[(value_index-j) % VALUE_BUFFER_SIZE][btn] < comparison)
          return true;
      }
      return false;
    } else {
      // For a press, the latest samples must all be lower than the last reference comparison
      #ifdef ABS_COMPARISON
        comparison = btn_values_min[btn] - (btn_values_range[btn]>>3);
      #else
        comparison = btn_values[(value_index+1) % VALUE_BUFFER_SIZE][btn] - btn_values_range[btn];
      #endif
      for (int j=0; j<VALUE_BUFFER_SIZE-1; j++){
        if (btn_values[(value_index-j) % VALUE_BUFFER_SIZE][btn] > comparison)
          return false;
      }
      return true;
    }
  #endif
}

void loop() {
  unsigned long start_us = micros();
  bool calibrating = (!digitalRead(recal_switch_pin));
  if (calibrating)
    calibrate(!last_calibrating);
  else
    full_scan();

  for (int btn=0; btn<NUM_BTNS; btn++) {
    btn_values[value_index][btn] = btn_values_raw[0][btn];
    for (int j=1; j<NUM_SAMPLES; j++)
      btn_values[value_index][btn] += btn_values_raw[j][btn];
    btn_values[value_index][btn] >>= NUM_SAMPLES_SHIFT;

    if (calibrating){  // Set all of the values buffer to the current value
      for (int j=0; j<VALUE_BUFFER_SIZE-1; j++)
        btn_values[(value_index-j) % VALUE_BUFFER_SIZE][btn] = btn_values[value_index][btn];
      btn_last_states[btn] = false;
      btn_values_cal[btn] = btn_values[value_index][btn];
    } else {  // Check for fall (if looking for press) or rise (if looking for release)
      btn_last_states[btn] = evaluate_button_state(btn);
    }
    Joystick.button(btn+1, btn_last_states[btn]);
  }
  #ifdef JOYSTICK_AXES
    Joystick.X(btn_values[value_index][0]>>2);
    Joystick.Y(btn_values[value_index][1]>>2);
    Joystick.Xrotate(btn_values[value_index][2]>>2);
    Joystick.Yrotate(btn_values[value_index][3]>>2);
    Joystick.sliderLeft(btn_values[value_index][4]>>2);
    Joystick.sliderRight(btn_values[value_index][5]>>2);
  #endif
  value_index = (value_index + 1) % VALUE_BUFFER_SIZE;
  last_calibrating = calibrating;

  Joystick.send();

  #ifdef SERIAL_MONITOR
    char str[130];
    sprintf(str, "Buttons: %d  %d  %d  %d  %d  %d", btn_values[0], btn_values[1], btn_values[2], btn_values[3], btn_values[4], btn_values[5]);
    CompositeSerial.println(str);
  #endif
  //Joystick.getOutput();  // We don't know anything about the rcv buffer. Would be useful for calibration process.
  while ((unsigned long)(micros() - start_us) < us_per_loop) {};  // We want consistent loop timing
}
