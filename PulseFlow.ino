/*
 * This file is part of PulseFlow.
 *
 * Copyright (C) 2024 Huib Smeets (pulseflow[at]huibsmeets[dot]com)
 *
 * PulseFlow is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * PulseFlow is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with [Project Name]. If not, see <https://www.gnu.org/licenses/>.
 */

#include <ArduinoBLE.h>
#include <mbed.h>

// There is no sensor pairing functionality implemented as only one
// HR sensor is active while using an indoor trainer in private spaces.
// However some trainers have a HR bridge function.
// To prevent connecting to the trainer HR service, the HR sensor to connect to is hardcoded.
// Your HR sensor name can be obtained by the NRF Connect app or your trainer software (Zwift, Rouvy, etc)

#define SENSORNAME "HRM-Dual:560827"

// To regulate the speed of the fan a PWM controlled AC dimmer used, sourced on AliExpress.
// Tests showed that this AC dimmer works better at a lower PWM frequency than the standard Arduino PWM frequency
// Instead of analogWriye() a MBED OS PWM function is used to generate a 250Hz PWM

mbed::PwmOut* pwmOnD6 = new mbed::PwmOut(digitalPinToPinName(D6));

// Number of heartrate samples to smooth single readings.
// The sensor is read once per second and the fanspeed is adjusted once per HRSAMPLES seconds.
// The fan has some inertia of it's own, more frequent adjustments have little use.

#define HRSAMPLES 5

// HR is mapped to the PWM duty-cycle with the Arduino map() function
// A midpoint value is available to alter the steepness of the linear map at some point
// HR values in bpm, duty-cycle in percentage 0-100% (because of interger function map())
// Adjust to your own HR rates and non-linearities of the fan.

#define MINHR 100
#define MIDHR 140
#define MAXHR 160
#define MINFAN 25
#define MIDFAN 50
#define MAXFAN 100

unsigned long currentMillis;

// Used for non-blocking(delay) BLUE LED blinking to signal periphical searching

unsigned long previousMillisLED = 0;
bool ledState = false;

// Workaround for an ArduinoBLE issue, see below.

unsigned long previousMillisBLE = 0;


void setup() {

  //Set the PWM frequency to 250Hz (4ms), set duty-cycle to 0 -> Fan is off

  pwmOnD6->period_ms(4);
  pwmOnD6->write(0.0f);

  //Set the builtin RGB LED's ports to Output, but this turns the LED's on too
  //as the logic is inverted due to the NANO 33 BLE hardware design.
  //Switch the LED of by setting the port to HIGH
  //FIXME: explicit OUTPUT setting needed?

  pinMode(LEDR, OUTPUT);
  digitalWrite(LEDR, HIGH);
  pinMode(LEDB, OUTPUT);
  digitalWrite(LEDB, HIGH);
  pinMode(LEDG, OUTPUT);
  digitalWrite(LEDG, HIGH);

  // Initialize the BLE hardware/environment/library,
  // if fatal error turn on the RED LED and loop forever.

  if (!BLE.begin()) {
    digitalWrite(LEDR, LOW);
    while (1)
      ;
  };

  // start scanning for a periphiral advertising the Heart Rate Measurement Service (GATT: UID=180D)

  BLE.scanForUuid("180D");
}

void loop() {

  // Obtain a periphiral advertising the HR sevice from the scan function

  BLEDevice peripheral = BLE.available();

  // Check if a periphiral is found an of the name matches the sensorname

  if (peripheral && peripheral.localName() == SENSORNAME) {

    // End scanning to save on resouces

    BLE.stopScan();

    // The sensor has been found, turn on the blue LED

    digitalWrite(LEDB, LOW);

    // The "main business" loop of reading sensor values and adjusting the fan speed
    // until the sensor is no longer connected

    HRM2FAN(peripheral);

    // No sensor connected anymore, start searching for a sensor agian

    previousMillisBLE = 0;
    BLE.scanForUuid("180D");

  } else {

    // As long as no sensor found blink the Blue led.
    // No blocking delay used as it interferes with the BLE.scan function.

    currentMillis = millis();
    if (currentMillis - previousMillisLED >= 500) {
      previousMillisLED = currentMillis;
      ledState = !ledState;
      digitalWrite(LEDB, ledState ? HIGH : LOW);
    }

    // There seems to be an issue with the BLE.scan function and/or Nano 33 BLE Rev2.
    // After about 5 to 10 minutes the BLE.scan function seems to break as it does no longer find any sensor.
    // This was noticed during a coffee break between two sessions on the trainer.
    // A workaround is to stop and start the BLE environment every 5 minutes

    if (currentMillis - previousMillisBLE >= 300000) {
      previousMillisBLE = currentMillis;
      BLE.end();
      if (!BLE.begin()) {
        digitalWrite(LEDR, LOW);
        while (1)
          ;
      };
      BLE.scanForUuid("180D");
    }
  }
}

void HRM2FAN(BLEDevice peripheral) {

  byte HRMcharvalue[2];
  uint8_t HRsampleValue[HRSAMPLES];
  int HRsampleCount;
  int FanDutyCycle;
  int HRavg;

  // Connect to the found peripheral, if not exit and start searching for a periphiral again.

  if (!peripheral.connect()) {
    return;
  }

  // Check for Attributes of the periphiral, if none found, disconnect, return to search again

  if (!peripheral.discoverAttributes()) {
    peripheral.disconnect();
    return;
  }

  // Retrieve the hearth rate characteristic (2A37), if none found, disconnect, return to search again

  BLECharacteristic Characteristic = peripheral.characteristic("2A37");
  if (!Characteristic) {
    peripheral.disconnect();
    return;
  }

  // The heart rate characteristic is a 'Notify' and must be subscribed to, to get values
  // If not able to subscribe, disconnect, return to search again

  if (!Characteristic.subscribe()) {
    peripheral.disconnect();
    return;
  };

  // A connection is established, turn blue LED off, Turn green LED on

  digitalWrite(LEDG, LOW);
  digitalWrite(LEDB, HIGH);

  // Inititalize the array with HR readings to zero

  for (int i = 0; i < HRSAMPLES; i++) HRsampleValue[i] = 0;
  HRsampleCount = 0;

  // As long a the sensor is connected readout the HR from the sensor

  while (peripheral.connected()) {

    // The characterisc 2A37 is of type Notify, check if a notification was send

    if (Characteristic.valueUpdated()) {

      //FIXME: is this needed?

      Characteristic.read();

      // Values a send as a variable length byte array
      // In case of the hearth rate it can be 2 to 512 bytes long
      // The first byte contains 8 flags (5 defined, 3 for future use, see GATT specs)
      // the first bit in the flag byte defines if the heart rate is send in 1 or 2 bytes, 8 or 16 interger
      // However this Arduino sketch takes a short-cut: it ignores this 1 or 2 bytes possibility as the Garmin HR sensor (and may others)
      // only send the HR as 1 byte value.

      Characteristic.readValue(HRMcharvalue, 2);
      HRsampleValue[HRsampleCount] = HRMcharvalue[1];
      HRsampleCount++;

      // When enough HR samples have been read, calculate the average, map the HR to the duty cycle, adjust the fan speed

      if (HRsampleCount >= HRSAMPLES) {

        // Average HR calculation

        HRavg = 0;
        for (int i = 0; i < HRSAMPLES; i++) HRavg += HRsampleValue[i];
        HRavg = HRavg / HRSAMPLES;

        // Map the HR to a dutycycle, low to mid and mid to high ranges

        if (HRavg >= MINHR && HRavg <= MIDHR) {
          FanDutyCycle = map(HRavg, MINHR, MIDHR, MINFAN, MIDFAN);
        }
        if (HRavg > MIDHR && HRavg <= MAXHR) {
          FanDutyCycle = map(HRavg, MIDHR, MAXHR, MIDFAN, MAXFAN);
        }

        // the map function returns out of range values when the input is also out of the set range
        // if HR is outside the map range, set the duty cycle to its minimum of maximum

        if (HRavg < MINHR) {
          FanDutyCycle = MINFAN;
        }
        if (HRavg > MAXHR) {
          FanDutyCycle = MAXFAN;
        }

        // update the duty cycle, adjust the speed of the fan

        pwmOnD6->write(FanDutyCycle / 100.0f);  //division must result in a float

        // clear the array with HR samples

        HRsampleCount = 0;
        for (int i = 0; i < HRSAMPLES; i++) HRsampleValue[i] = 0;
      }
    }

    delay(1000);
  }

  // connection lost, set the fan to it's lowest defined speed, switch of the green LED

  pwmOnD6->write(MINFAN / 100.0f);
  digitalWrite(LEDG, HIGH);
}
