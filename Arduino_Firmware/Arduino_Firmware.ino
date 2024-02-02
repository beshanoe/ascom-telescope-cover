/*
 * Telescope_Cover_Firmware.ino
 * Copyright (C) 2022 - Present, Julien Lecomte - All Rights Reserved
 * Licensed under the MIT License. See the accompanying LICENSE file for terms.
 */

#include <ESP32Servo.h>
#include "ServoEasing.hpp"
#include <Wire.h>
#include <ADXL345.h>
#include <AceButton.h>

constexpr auto DEVICE_GUID = "cbbada80-325b-459a-9f70-8677a3e31241";

constexpr auto COMMAND_PING = "COMMAND:PING";
constexpr auto RESULT_PING = "RESULT:PING:OK:";

constexpr auto COMMAND_INFO = "COMMAND:INFO";
constexpr auto RESULT_INFO = "RESULT:DarkSkyGeek's Telescope Cover Firmware v1.0";

constexpr auto COMMAND_GFORCE = "COMMAND:GFORCE:";

constexpr auto COMMAND_COVER = "COMMAND:COVER";
constexpr auto COMMAND_SET_COVER_ON = "COMMAND:SET_COVER:ON";
constexpr auto COMMAND_SET_COVER_OFF = "COMMAND:SET_COVER:OFF";

constexpr auto COMMAND_LIGHT = "COMMAND:LIGHT";
constexpr auto COMMAND_SET_LIGHT_ON = "COMMAND:SET_LIGHT:ON";
constexpr auto COMMAND_SET_LIGHT_OFF = "COMMAND:SET_LIGHT:OFF";

constexpr auto COMMAND_LIGHT_INTENSITY = "COMMAND:LIGHT:INTENSITY";
constexpr auto COMMAND_LIGHT_SET_INTENSITY = "COMMAND:LIGHT:SET_INTENSITY:";

constexpr auto ERROR_INVALID_COMMAND = "ERROR:INVALID_COMMAND";

#define MIN_BRIGHTNESS 0
#define MAX_BRIGHTNESS 255
#define PWM_FREQ 20000
#define BUTTON1_PIN A0
#define BUTTON2_PIN A1
#define BUTTON3_PIN A2

using namespace ace_button;

byte brightness = 0;
int servoPin = 9;
int ledPin = 8;
int servoSpeed = 90;

ESP32PWM ledPWM;
ServoEasing servo;
ADXL345 adxl;

AceButton firstButton(BUTTON1_PIN);
AceButton secondButton(BUTTON2_PIN);
AceButton thirdButton(BUTTON3_PIN);

void handleButtonEvent(AceButton*, uint8_t, uint8_t);

enum CoverState {
  coverOpen,
  coverClosed
} state;

// The `setup` function runs once when you press reset or power the board.
void setup() {
  state = coverClosed;

  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  // Initialize serial port I/O.
  Serial.begin(57600);
  // while (!Serial) {
  //   ;  // Wait for serial port to connect. Required for native USB!
  // }
  Serial.flush();

  setupADXL345();

  // Initialize servo.
  // Important: We assume that the cover is in the closed position!
  // If it's not, then the servo will brutally close it when the system is powered up!
  // That may damage the mechanical parts, so be careful...
  servo.attach(servoPin, 0, 510, 2440);

  // Make sure the RX, TX, and built-in LEDs don't turn on, they are very bright!
  // Even though the board is inside an enclosure, the light can be seen shining
  // through the small opening for the USB connector! Unfortunately, it is not
  // possible to turn off the power LED (green) in code...
  // pinMode(PIN_LED_TXL, INPUT);
  // pinMode(PIN_LED_RXL, INPUT);
  // pinMode(LED_BUILTIN, OUTPUT);
  // digitalWrite(LED_BUILTIN, HIGH);

  // Setup LED pin as output
  ledPWM.attachPin(ledPin, PWM_FREQ, 10);

  pinMode(BUTTON1_PIN, INPUT);
  pinMode(BUTTON2_PIN, INPUT);
  pinMode(BUTTON3_PIN, INPUT);

  firstButton.setEventHandler(handleButtonEvent);
  secondButton.setEventHandler(handleButtonEvent);
  thirdButton.setEventHandler(handleButtonEvent);
}

void handleButtonEvent(AceButton* button, uint8_t eventType,
                       uint8_t /*buttonState*/) {
  switch (eventType) {
    case AceButton::kEventPressed:
      if (button->getPin() == BUTTON1_PIN) {
        if (state == coverOpen) {
          closeCover();
        } else {
          openCover();
        }
      }
      if (button->getPin() == BUTTON2_PIN) {
        if (brightness > 0) {
          calibratorOff();
        } else {
          calibratorOn(100);
        }
      }
      if (button->getPin() == BUTTON3_PIN) {
        brightness += 20;
        if (brightness > 255) {
          calibratorOff();
        } else {
          calibratorOn(brightness);
        }
      }
      break;
  }
}

void setupADXL345() {
  adxl.powerOn();

  //set activity/ inactivity thresholds (0-255)
  adxl.setActivityThreshold(75);    //62.5mg per increment
  adxl.setInactivityThreshold(75);  //62.5mg per increment
  adxl.setTimeInactivity(10);       // how many seconds of no activity is inactive?

  //look of activity movement on this axes - 1 == on; 0 == off
  adxl.setActivityX(1);
  adxl.setActivityY(1);
  adxl.setActivityZ(1);

  //look of inactivity movement on this axes - 1 == on; 0 == off
  adxl.setInactivityX(1);
  adxl.setInactivityY(1);
  adxl.setInactivityZ(1);

  //look of tap movement on this axes - 1 == on; 0 == off
  adxl.setTapDetectionOnX(0);
  adxl.setTapDetectionOnY(0);
  adxl.setTapDetectionOnZ(1);

  //set values for what is a tap, and what is a double tap (0-255)
  adxl.setTapThreshold(50);      //62.5mg per increment
  adxl.setTapDuration(15);       //625us per increment
  adxl.setDoubleTapLatency(80);  //1.25ms per increment
  adxl.setDoubleTapWindow(200);  //1.25ms per increment

  //set values for what is considered freefall (0-255)
  adxl.setFreeFallThreshold(7);  //(5 - 9) recommended - 62.5mg per increment
  adxl.setFreeFallDuration(45);  //(20 - 70) recommended - 5ms per increment

  //setting all interrupts to take place on int pin 1
  //I had issues with int pin 2, was unable to reset it
  adxl.setInterruptMapping(ADXL345_INT_SINGLE_TAP_BIT, ADXL345_INT1_PIN);
  adxl.setInterruptMapping(ADXL345_INT_DOUBLE_TAP_BIT, ADXL345_INT1_PIN);
  adxl.setInterruptMapping(ADXL345_INT_FREE_FALL_BIT, ADXL345_INT1_PIN);
  adxl.setInterruptMapping(ADXL345_INT_ACTIVITY_BIT, ADXL345_INT1_PIN);
  adxl.setInterruptMapping(ADXL345_INT_INACTIVITY_BIT, ADXL345_INT1_PIN);

  //register interrupt actions - 1 == on; 0 == off
  adxl.setInterrupt(ADXL345_INT_SINGLE_TAP_BIT, 1);
  adxl.setInterrupt(ADXL345_INT_DOUBLE_TAP_BIT, 1);
  adxl.setInterrupt(ADXL345_INT_FREE_FALL_BIT, 1);
  adxl.setInterrupt(ADXL345_INT_ACTIVITY_BIT, 1);
  adxl.setInterrupt(ADXL345_INT_INACTIVITY_BIT, 1);
}

// The `loop` function runs over and over again until power down or reset.
void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    if (command == COMMAND_PING) {
      handlePing();
    } else if (command == COMMAND_INFO) {
      sendFirmwareInfo();
    } else if (command == COMMAND_COVER) {
      sendCurrentCover();
    } else if (command == COMMAND_SET_COVER_ON) {
      openCover();
    } else if (command == COMMAND_SET_COVER_OFF) {
      closeCover();
    } else if (command == COMMAND_LIGHT) {
      sendLightState();
    } else if (command == COMMAND_LIGHT_INTENSITY) {
      sendCalibratorBrightness();
    } else if (command == COMMAND_SET_LIGHT_ON) {
      calibratorOn(100);
    } else if (command == COMMAND_SET_LIGHT_OFF) {
      calibratorOff();
    } else if (command.startsWith(COMMAND_LIGHT_SET_INTENSITY)) {
      String arg = command.substring(strlen(COMMAND_LIGHT_SET_INTENSITY));
      byte value = (byte)arg.toInt();
      calibratorOn(value);
    } else if (command.startsWith(COMMAND_GFORCE)) {
      sendInfoG(command);
    } else {
      handleInvalidCommand();
    }
  }

  firstButton.check();
  secondButton.check();
  thirdButton.check();
}


//-- CALIBRATOR HANDLING ------------------------------------------------------

void sendLightState() {
  Serial.print("RESULT:");
  Serial.print(COMMAND_LIGHT);
  Serial.print(":");
  if (brightness > 0) {
    Serial.println("ON");
  } else {
    Serial.println("OFF");
  }
}

void sendCalibratorBrightness() {
  Serial.print("RESULT:");
  Serial.print(COMMAND_LIGHT_INTENSITY);
  Serial.print(":");
  Serial.println(brightness);
}

void setBrightness() {
  // This only works on Seeeduino Xiao.
  // The `pwm` function is defined in the following file:
  // %localappdata%\Arduino15\packages\Seeeduino\hardware\samd\1.8.2\cores\arduino\wiring_pwm.cpp
  // For other Arduino-compatible boards, consider using:
  // analogWrite(ledPin, brightness);
  // The nice thing about the `pwm` function is that we can set the frequency
  // to a much higher value (I use 20kHz) This does not work on all pins!
  // For example, it does not work on pin 7 of the Xiao, but it works on pin 8.
  float value = mapf(brightness, MIN_BRIGHTNESS, MAX_BRIGHTNESS, 0, 1);
  ledPWM.writeScaled(value);
}

void calibratorOn(byte _brightness) {
  brightness = _brightness;
  setBrightness();
}

void calibratorOff() {
  brightness = 0;
  setBrightness();
}

//-- TELESCOPE COVER HANDLING ------------------------------------------------

void handlePing() {
  Serial.print(RESULT_PING);
  Serial.println(DEVICE_GUID);
}

void sendFirmwareInfo() {
  Serial.println(RESULT_INFO);
}

void sendInfoG(String command) {
  int x, y, z;
  adxl.readXYZ(&x, &y, &z);  //read the accelerometer values and store them in variables  x,y,z
  double xyz[3];
  double ax, ay, az;
  adxl.getAcceleration(xyz);
  ax = xyz[0];
  ay = xyz[1];
  az = xyz[2];

  String arg = command.substring(strlen(COMMAND_GFORCE));
  Serial.print("RESULT:");
  Serial.print(command);
  Serial.print(":");
  switch (arg[0]) {
    case 'X':
      Serial.println(ax);
      break;
    case 'Y':
      Serial.println(ay);
      break;
    case 'Z':
      Serial.println(az);
      break;
    default:
      Serial.println(0);
  }
}

void sendCurrentCover() {
  Serial.print("RESULT:");
  Serial.print(COMMAND_COVER);
  Serial.print(":");
  if (state == coverOpen) {
    Serial.println("ON");
  } else {
    Serial.println("OFF");
  }
}

void openCover() {
  servo.startEaseTo(180, servoSpeed);

  state = coverOpen;
}

void closeCover() {
  servo.startEaseTo(0, servoSpeed);

  state = coverClosed;
}

void handleInvalidCommand() {
  Serial.println(ERROR_INVALID_COMMAND);
}