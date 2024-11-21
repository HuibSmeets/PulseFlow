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

// There is no sensor pairing functionality implemented under the assumption that there is only 1
// HR sensor active while using the indoor trainer in private spaces.
// However some trainers have a HR bridge function.
// To prevent connecting to the trainer HR service the name of the sensor to use is hardcoded (and unique)
// Your HR sensor name can be obtained by using the NRF Connect app or via your trainer software (Zwift, Rouvy, etc)

#define SENSORNAME "HRM-Dual:560827"

// To regulate the speed of the fan a PWM controlled AC dimmer is used, sourced from AliExpress.
// Tests showed that this AC dimmer works better at a lower PWM frequency than the standard Arduino PWM frequency
// Instead of analogWrite() a MBED OS PWM function is used to generate a 250Hz PWM, therefore that piece of code is
// Nano 33 BLE Rev 2 specific

mbed::PwmOut* pwmOnD6 = new mbed::PwmOut(digitalPinToPinName(D6));

// The number of heartrate samples to smooth single readings.
// The sensor is read once per second and the fanspeed is adjusted once per HRSAMPLES seconds.
// The fan has some inertia of it's own, more frequent adjustments have little use and is not too slow

#define HRSAMPLES 5

// HR is mapped to the PWM duty-cycle with the Arduino map() function
// A midpoint value is available to alter the steepness of the linear map at some point
// HR values in bpm, duty-cycle in percentage 0-100% (because of integer calculation inside function map())
// Adjust these 6 values to your own heart rates (and non-linearities of the fan).

#define MINHR 100
#define MIDHR 140
#define MAXHR 160
#define MINFAN 25
#define MIDFAN 50
#define MAXFAN 100

unsigned long currentMillis;

// Used for non-blocking(delay) blue LED blinking to signal peripheral searching

unsigned long previousMillisLED = 0;
bool ledState = false;

// Workaround for an ArduinoBLE issue, see below.

unsigned long previousMillisBLE = 0;


void setup() {

  //Set the PWM frequency to 250Hz (4ms), set duty-cycle to 0 -> Fan is off

  pwmOnD6->period_ms(4);
  pwmOnD6->write(0.0f);

  //Set the builtin RGB LED's ports to Output, but this turns the LED's on too!
  //as the On/Off logic is inverted due to the NANO 33 BLE hardware design.
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

  // start scanning for a peripheral advertising the Heart Rate Measurement Service (GATT: UID=180D)

  BLE.scanForUuid("180D");
}

void loop() {

  // Check if a peripheral is found and its name matches the sensorname, if not: continue scanning

  BLEDevice peripheral = BLE.available();
  if (peripheral && peripheral.localName() == SENSORNAME) {

    // End scanning to save on resouces

    BLE.stopScan();

    // The sensor has been found, turn on the blue LED

    digitalWrite(LEDB, LOW);

    // The "main business" loop of reading sensor values and adjusting the fan speed
    // until the sensor is no longer connected

    HRM2FAN(peripheral);

    // The sensor is not connected anymore, start searching again

    previousMillisBLE = 0;
    BLE.scanForUuid("180D");

  } else {

    // As long as no sensor is found blink the blue led.
    // No delay() is used as it interferes with the BLE.scan() function.

    currentMillis = millis();
    if (currentMillis - previousMillisLED >= 500) {
      previousMillisLED = currentMillis;
      ledState = !ledState;
      digitalWrite(LEDB, ledState ? HIGH : LOW);
    }

    // There seems to be an issue with the BLE.scan function and/or Nano 33 BLE Rev2.
    // After about 5 to 10 minutes of scanning for peripherals the BLE.scan function does not work anymore.
    // The workaround is to stop and start the BLE environment every 5 minutes

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

  // Connect to the found peripheral, if not able to connect: exit and start searching for a periphiral again.

  if (!peripheral.connect()) {
    return;
  }

  // Check for attributes of the periphiral, if none found: disconnect, return to search again

  if (!peripheral.discoverAttributes()) {
    peripheral.disconnect();
    return;
  }

  // Retrieve the heart rate characteristic (2A37), if not available: disconnect, return to search again

  BLECharacteristic Characteristic = peripheral.characteristic("2A37");
  if (!Characteristic) {
    peripheral.disconnect();
    return;
  }

  // The heart rate characteristic is a 'Notify' and must be subscribed to, to get values
  // If not able to subscribe: disconnect, return to search again

  if (!Characteristic.subscribe()) {
    peripheral.disconnect();
    return;
  };

  // A connection is established, turn the blue LED off and green LED on

  digitalWrite(LEDG, LOW);
  digitalWrite(LEDB, HIGH);

  // Inititalize the array with HR samples to zero

  for (int i = 0; i < HRSAMPLES; i++) HRsampleValue[i] = 0;
  HRsampleCount = 0;

  // As long a the sensor is connected: read the HR from the sensor

  while (peripheral.connected()) {

    // The characterisc 2A37 is of type Notify, check if a notification was send by the sensor

    if (Characteristic.valueUpdated()) {

      //FIXME: is this read needed?

      Characteristic.read();

      // Sensor attribute values are send as a variable length byte array
      // In case of the hearth rate it can be 2 to 512 bytes long
      // The first byte contains 8 flags (5 defined, 3 for future use, see GATT specs)
      // the first bit in the flag byte defines if wether the heart rate is send in 1 or 2 bytes, 8 or 16 interger.
      // This Arduino sketch takes a short-cut: it ignores this all as the Garmin sensor only uses 1 byte to send the HR value

      Characteristic.readValue(HRMcharvalue, 2);
      HRsampleValue[HRsampleCount] = HRMcharvalue[1]; //retrieve the HR from the 2nd byte
      HRsampleCount++;

      // When the set number of HR samples have been read: calculate the average, map the HR to the duty cycle, adjust the fan speed

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
        // if the HR is outside the map range, set the duty cycle to its minimum or maximum

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
