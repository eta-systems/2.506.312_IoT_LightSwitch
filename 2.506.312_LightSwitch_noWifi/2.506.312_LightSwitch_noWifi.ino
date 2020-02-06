/************************************************************************************************
 * @file     2.506.312_LightSwitch_noWifi.ino
 * @author   Simon Burkhardt (simon.burkhardt@eta-systems.ch)
 * @date     2020.02.06
 * @copy     (c) 2020 eta Systems GmbH
 * @brief    test Encoder Library with PCA PWM driver
 * @details  Erkenntnis: - Encoder ISRs sollen keinen check des Status machen,
 *                          da diese bis zu 8x pro drehtakt ausgelöst werden
 *                       - implementierung wie unten funktioniert genügend aber noch nicht befriedigend
 *                       - es wurden keine Kondensatoren an den Encoder Pins angebracht
 *                       - 220nF Cap an Button Pin
 *                       - LEDs on-board: pwm.setPWM(i+8, 0, x);   off: x = 4095 / on: x = 0
 *                       - Silent LED:    pwm.setPWM(i  , 0, x);   off: x =      / on: x = 
 * 
 * @Todo: LED brightness matching curve: Silent LED vs. on-board LED
 * @Todo: Timout back to IDLE
 * 
 * @dependencies - Arduino 1.8.10
 *               - esp8266 Core 2.6.2
 *               - https://github.com/enjoyneering/RotaryEncoder   (GPLv3)
 *               - https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library (BSD)
 * 
 ************************************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <RotaryEncoder.h>

/* Private typedef -----------------------------------------------------------*/
#define SDA_PIN 2
#define SCL_PIN 14
#define ROTARY_PIN1 4
#define ROTARY_PIN2 5
#define BUTTON_PIN  13

#define LED_ON  0
#define LED_OFF 4095

enum{ST_IDLE, ST_SCROLL, ST_DIMM};

/* Private variables ---------------------------------------------------------*/
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x47, Wire);
RotaryEncoder encoder(ROTARY_PIN1, ROTARY_PIN2, BUTTON_PIN);

volatile int16_t enc_position = 0;
volatile int16_t enc_difference = 0;

volatile int8_t state = ST_IDLE;    // initial state
volatile int16_t led_states[8];     // PWM memory
volatile uint8_t selected_led  = 0;
volatile uint16_t selected_pwm = 0;


/* Private function prototypes -----------------------------------------------*/
//interrupt service routines need to be in ram
void ICACHE_RAM_ATTR encoderISR() { 
  encoder.readAB();          // update encoder state
}

void ICACHE_RAM_ATTR encoderButtonISR() {
  encoder.readPushButton();  // update button state
}

void encoderChanged(void){
  Serial.println(enc_position);
  if(state == ST_SCROLL){
    // Minus to change rotation direction
    selected_led = (selected_led - enc_difference) % 8;
  }
  if(state == ST_DIMM){
    selected_pwm = (selected_pwm + (enc_difference * 64));
  }
  enc_difference = 0;  // reset value;

  if(selected_pwm > 4095)
    selected_pwm = 4095;
  
}

void buttonPressed(void){
  Serial.println(F("PRESSED"));
  if(state == ST_IDLE){
    state = ST_SCROLL;
  } else if(state == ST_SCROLL){
    state = ST_DIMM;
    selected_pwm = led_states[selected_led];
  } else {
    state = ST_IDLE;
  }
}

/* IDLE STATE: just display the brightness */
void stateIdle(void){
  for(uint8_t i=0; i<8; i++){
    pwm.setPWM(i  , 0, led_states[i]);  // SilendLED on Port i
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
      pwm.setPWM(i  , 0, led_states[i]);  // SilendLED on Port i
      pwm.setPWM(i+8, 0, lampToLedBrightness(led_states[i]));  // on board LEDs
    } else {
      /** @Todo: Should on-board LEDs be off while dimming? */
      pwm.setPWM(i  , 0, led_states[i]);  // SilendLED on Port i
      pwm.setPWM(i+8, 0, lampToLedBrightness(led_states[i]));  // on board LEDs
    }
  }
}

uint16_t lampToLedBrightness(uint16_t pwm){
  /** @Todo: brightness curve */
  // return input value for 1:1 relation
  return pwm;
}

/* Private user code ---------------------------------------------------------*/
void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.println(F("\n\neta-systems\nIoT Light Switch"));
  
  Wire.begin(SDA_PIN, SCL_PIN);  // initialize I2C with custom SDA/SCL Pins
  delay(50);

  pwm.begin();         // start PWM driver chip
  encoder.begin();     // start Endocer Library & add GPIO listeners
  attachInterrupt(digitalPinToInterrupt(ROTARY_PIN1), encoderISR,       CHANGE); 
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN),  encoderButtonISR, FALLING);

  for(uint8_t i=0; i<8; i++){
    led_states[i] = LED_OFF;
  }
}

/* Infinite loop -------------------------------------------------------------*/
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
  
  delay(1);
}
