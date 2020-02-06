/************************************************************************************************
 * @file     2.506.312_LightSwitch_noWifi.ino
 * @author   Simon Burkhardt (simon.burkhardt@eta-systems.ch)
 * @date     2020.02.06
 * @copy     (c) 2020 eta Systems GmbH
 * @brief    Silent LED dimmer interface with ADC feedback
 * @details  Erkenntnis: - Encoder ISRs must not do status check,
 *                          since ISRs are called up to 8x per encoder step
 *                       - implementation is working
 *                       - no capacitors on encoder pins
 *                       - 220nF Cap on Button Pin
 *                       - LEDs on-board: pwm.setPWM(i+8, 0, x);   off: x = 4095 / on: x = 0
 *                       - Silent LED:    pwm.setPWM(i  , 0, x);   off: x = 0    / on: x = 1500
 *                       - Silent LED: DIP Switch: 1=on / 2=on / 3=off
 *                       - compiled binaries are created in build path, 
 *                          set paht with build.path=C:\Arduino-Output in preferences.txt
 * 
 * @Todo: LED brightness matching curve: Silent LED vs. on-board LED
 * @Todo: Timout back to IDLE when user is not changing the brightness
 * @Todo: Silent LED starts to flicker for pwm value of:  pwm.setPWM(i, 0, 4095);
 * @Todo: Silent LED has max brightness at around pwm.setPWM(i, 0, 1550); 
 * @Todo: Temperature Measurement reference
 *  
 * @dependencies - Arduino 1.8.10
 *               - esp8266 Core 2.6.2
 *               - https://github.com/enjoyneering/RotaryEncoder   (GPLv3)
 *               - https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library (BSD)
 *               - https://github.com/andriyadi/ESP32-TLA2024 (MIT)
 * 
 * 
 * Mapping on the PWM driver: each on board LED corresponds to one Silent LED
 * On Board LED : 08 09 10 11 12 13 14 15
 * Silen LED    : 07 06 05 04 03 02 01 00
 ************************************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <RotaryEncoder.h>
#include <TLA2024.h>

/* Private typedef -----------------------------------------------------------*/
#define ADDRESS_PWM 0x47
#define ADDRESS_ADC 0xC8

#define SDA_PIN 2
#define SCL_PIN 14
#define ROTARY_PIN1 4
#define ROTARY_PIN2 5
#define BUTTON_PIN  13

#define LED_ON  0
#define LED_OFF 4095

#define MAX_PWM 1500

enum{ST_IDLE, ST_SCROLL, ST_DIMM};

/* Private variables ---------------------------------------------------------*/
TLA2024 adc = TLA2024(&Wire);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(ADDRESS_PWM, Wire);
RotaryEncoder encoder(ROTARY_PIN1, ROTARY_PIN2, BUTTON_PIN);

volatile int16_t enc_position = 0;
volatile int16_t enc_difference = 0;

volatile int8_t  state = ST_IDLE;    // initial state
volatile int16_t led_states[8];      // PWM memory
volatile int8_t  selected_led = 0;
volatile int16_t selected_pwm = 0;


/* Private function prototypes -----------------------------------------------*/
// interrupt service routines need to be in ram
// don't do anything else in here besides readAB / readPushButton
void ICACHE_RAM_ATTR encoderISR() { 
  encoder.readAB();          // update encoder state
}

void ICACHE_RAM_ATTR encoderButtonISR() {
  encoder.readPushButton();  // update button state
}

// if encoder value has changed (polling in loop)
// change value depending on state machine
void encoderChanged(void){
  if(state == ST_SCROLL){
    selected_led = (selected_led + enc_difference) % 8;
    Serial.print(F("LED: ")); Serial.println(selected_led);
  }
  if(state == ST_DIMM){
    selected_pwm = (selected_pwm + (enc_difference * 64));
    // boundry check, don't over- / underflow
    if(selected_pwm > MAX_PWM)
      selected_pwm = MAX_PWM;
    if(selected_pwm < 0)
      selected_pwm = 0;
    Serial.print(F("pwm: ")); Serial.println(selected_pwm);
  }
  enc_difference = 0;  // reset value;
}

// if button is pressed (polling in loop)
// only change state of state machine
void buttonPressed(void){
  if(state == ST_IDLE){
    state = ST_SCROLL;
    Serial.println(F("-SCROLL"));
  } else if(state == ST_SCROLL){
    state = ST_DIMM;
    Serial.println(F("-DIMM"));
    selected_pwm = led_states[selected_led]; // start with current brightness
  } else {
    state = ST_IDLE;
    Serial.println(F("-IDLE"));
  }
}

/* IDLE STATE: just display the brightness */
void stateIdle(void){
  for(uint8_t i=0; i<8; i++){
    pwm.setPWM(7-i  , 0, led_states[i]);  // SilendLED on Port i
    pwm.setPWM(i+8, 0, lampToLedBrightness(led_states[i]));  // on board LEDs
  }
}

/* SCROLL STATE: scroll through lights until one is selected */
void stateScroll(void){
  /** @Todo: should the real lights also be turned off for scrolling? */
  for(uint8_t i=0; i<8; i++){
    if(i == selected_led){
      pwm.setPWM(i+8, 0, LED_ON);
    } else {
      pwm.setPWM(i+8, 0, LED_OFF);
    }
  }
}

/* DIMM STATE: dimm selected light */
void stateDimm(void){
  for(uint8_t i=0; i<8; i++){
    if(i == selected_led){
      led_states[i] = selected_pwm;
      pwm.setPWM(7-i  , 0, led_states[i]);  // SilendLED on Port i
      pwm.setPWM(i+8, 0, lampToLedBrightness(led_states[i]));  // on board LEDs
    } else {
      /** @Todo: Should on-board LEDs be off while dimming? */
      pwm.setPWM(7-i  , 0, led_states[i]);  // SilendLED on Port i
      pwm.setPWM(i+8, 0, lampToLedBrightness(led_states[i]));  // on board LEDs
    }
  }
}

/** 
 *  on board LEDs and Silent LEDs don't have the same brightness curve
 *  with this function a brightness curve can be implemented to correct
 *  the two values
 */
uint16_t lampToLedBrightness(uint16_t pwm){
  /** @Todo: brightness curve */
  // return input value for 1:1 relation
  // uint16_t newp = pwm;
  
  // square relation x^2
  // uint16_t newp = (pwm*pwm)/4095;

  /**
   * Silent LED   reaches max brightness at pwm = 1500
   * on board LED reaches max brightness at pwm = 4095
   */
  uint16_t newp = pwm*4095/MAX_PWM;
  
  return 4095-newp;
}

/**
 * Use the on board TLA2024 ADC to read Vcc / Temperature and Current Draw
 * @Todo: use multiple samples to average the measurements / can the ADC do this internally?
 */
void updateADC(void){
  uint8_t adcn = 1;
  float val = 0.0f;
  float power = 0.0f;

  // Temperature - 500mV@0°C + 10mV/°C (MCP9700A)
  adcn = 2;
  val = (adc.voltageRead(adcn)-0.500f)/0.010f  ;
  Serial.printf("Temp  = %.3f °C\n", val);
  // VCC - Voltage Divider 1:1
  adcn = 1;
  val = 2.0f * adc.voltageRead(adcn);
  Serial.printf("Vcc   = %.3f V\n", val);
  // V LED (12V) - Voltage Divider 30k/4.7k
  adcn = 0;
  val = (30.0f + 4.7f)/4.7f * adc.voltageRead(adcn);
  power = val;
  Serial.printf("V_Led = %.3f V\n", val);
  // Current Draw - 10mR Shunt / 50V/V gain (INA180A2IDBVR)
  adcn = 3;
  val = 1000.0f / 0.01f / 50.0f * adc.voltageRead(adcn);
  power = power*val;
  Serial.printf("I_Led = %.3f mA\n", val);
  Serial.printf("P     = %.3f W\n", power);
}

/* Private user code ---------------------------------------------------------*/
void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.println(F("\n\neta-systems\nIoT Light Switch"));
  
  Wire.begin(SDA_PIN, SCL_PIN);  // initialize I2C with custom SDA/SCL Pins
  delay(50);

  if(!adc.begin(ADDRESS_ADC)) {
      Serial.println(F("could not init ADC"));
  }
  adc.setFullScaleRange(TLA2024::FSR_2_048V);
  adc.setDataRate(TLA2024::DR_3300SPS);
  adc.setOperatingMode(TLA2024::OP_SINGLE);

  pwm.begin();         // start PWM driver chip
  pwm.setOscillatorFrequency(27000000);  // The int.osc. is closer to 27MHz
  pwm.setPWMFreq(1600);
  pwm.setOutputMode(true);   // must be Totem-Pole to control the SilentLED
  
  encoder.begin();     // start Endocer Library & add GPIO listeners
  attachInterrupt(digitalPinToInterrupt(ROTARY_PIN1), encoderISR,       CHANGE); 
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN),  encoderButtonISR, FALLING);

  for(uint8_t i=0; i<8; i++){
    led_states[i] = 0;
  }
}

/* Infinite loop -------------------------------------------------------------*/

uint8_t ticker=0;

void loop() {
  // check if encoder has been rotated
  if (enc_position != encoder.getPosition()){
    enc_difference = encoder.getPosition() - enc_position;
    enc_position = encoder.getPosition();
    encoderChanged();
  }
  
  // check if button has been pressed
  if (encoder.getPushButton() == true)
    buttonPressed();

  switch(state){
    case ST_SCROLL:
      stateScroll();
      break;
    case ST_DIMM:
      stateDimm();
      break;
    case ST_IDLE:
    default:
      stateIdle();
  }

  // be careful with blocking functions in loop()
  delay(1);
  ticker ++;
  if(!ticker) // only update every ~1ms * 255
    updateADC();
}
