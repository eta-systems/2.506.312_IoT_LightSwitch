2.506.312 IOT Light Switch

![](https://img.shields.io/badge/License-GPLv3-blue.svg)

---


### Dependencies

- Arduino 1.8.10
- esp8266 Core 2.6.2

```c++
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <RotaryEncoder.h>
```

- [RotaryEncoder](https://github.com/enjoyneering/RotaryEncoder) (GPLv3)
- [Adafruit_PWMServoDriver](https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library) (BSD)
- [ESP32-TLA2024](https://github.com/andriyadi/ESP32-TLA2024)

### Hardware

**Pins**

```c++
#define SDA_PIN 2
#define SCL_PIN 14
#define ROTARY_PIN1 4
#define ROTARY_PIN2 5
#define BUTTON_PIN  13
```

**I2C Address**

```c++
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x47);

//    I2C device found at address 0x47  ! --> PCA9685 PWM driver
//    I2C device found at address 0x70  ! --> TLA2024 ADC
```

---

### Wichtig

- Encoder ISRs sollen keinen check des Status machen, da diese bis zu 8x pro drehtakt ausgelöst werden
- implementierung wie unten funktioniert befriedigend
- es wurden keine Kondensatoren an den Encoder Pins angebracht
- 220nF Cap an Button Pin
- LEDs on-board: pwm.setPWM(i+8, 0, x);   off: x = 4095 / on: x = 0
- Silent LED:    pwm.setPWM(i  , 0, x);   off: x = 0    / on: x = 1500
- Silent LED: DIP Switch: 1=on / 2=on / 3=off
- kompilierte Binaries werden in build path erstellt, einstellen unter build.path=C:\Arduino-Output in preferences.txt

### Todos

- LED brightness matching curve: Silent LED vs. on-board LED
- Timout back to IDLE
- Silent LED starts to flicker for pwm value of:  pwm.setPWM(i, 0, 4095);
- Silent LED has max brightness at around pwm.setPWM(i, 0, 1550);

### License

Since the Encoder Library is GPLv3, all other code must be released under GPLv3 also.

(c) 2020 eta Systems GmbH
