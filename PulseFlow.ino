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
#define MINFAN 30
#define MIDFAN 55
#define MAXFAN 100

// use #define the set the type of dimmer beeing used.
//    PWM: Dimmer controlled by PWM
//    ZCD: Dimmer that signals Zero-Crossing, sketch calculates the trigger delay

//#define PWM
#define ZCD

#define FANSTARTIMMEDIATE
#define FANSSTARTSPEED 30


unsigned long currentMillis;

// Used for non-blocking(delay) blue LED blinking to signal peripheral searching

unsigned long previousMillisLED = 0;
bool ledState = false;

// Workaround for an ArduinoBLE issue, see below.

unsigned long previousMillisBLE = 0;

#ifdef PWM

// To regulate the speed of the fan a PWM controlled AC dimmer is used, sourced from AliExpress.
// Tests showed that this AC dimmer works better at a lower PWM frequency than the standard Arduino PWM frequency
// Instead of analogWrite() a MBED OS PWM function is used to generate a 250Hz PWM, therefore that piece of code is
// Nano 33 BLE Rev 2 specific

mbed::PwmOut* pwmOnPin = new mbed::PwmOut(digitalPinToPinName(D6));

#endif

#ifdef ZCD

// To regulate the speed of the fan an AC dimmer with Zero Crossing detection is used, sourced from AliExpress.
// For delay timing of the triggering of the triac Nano 33 BLE Rev 2 specific code is used.

#include <nrf_timer.h>

// Pin definitions
const uint8_t interruptPin = 2;  // Pin to trigger the interrupt
const uint8_t triacPin = 6;      // Pin to trigger the TRIAC

// Constants
const uint32_t pulseWidthUs = 50;         // Pulse width in microseconds for the triac
const uint32_t halfSineDuration = 10000;  // half-sine wave duration in microseconds 50Hz: 10000us, 60Hz: 8333us

// Parameter for sine delay (adjustable between 0 and 100% of the sine)
volatile uint8_t dimmingPercentage = 0;

// Interrupt handler for Zero Crossing
void handleZC() {

  NVIC_DisableIRQ(TIMER3_IRQn);

  // in case dimming at the lowest and highest range: CPU cycles consume time which causes timing mis-align with the actual
  // zero crossing therefore at the lowest and highest end the dimmer is shutoff or full.
  // Timer is only used in the in-between values. Timer3: 1MHz --> Delay thus calculated in microseconds

  switch (dimmingPercentage) {
    case 0 ... 5:
      {
        digitalWrite(triacPin, LOW);
        break;
      }
    case 95 ... 100:
      {
        digitalWrite(triacPin, HIGH);
        break;
      }
    default:
      {
        NRF_TIMER3->TASKS_CLEAR = 1;
        NRF_TIMER3->CC[0] = halfSineDuration - (uint32_t)dimmingPercentage * halfSineDuration / 100.0;
        NRF_TIMER3->TASKS_START = 1;
        NVIC_EnableIRQ(TIMER3_IRQn);
      }
  }
}

// Timer3 Compare Irq Event Handler --> delay ended, trigger the triac

extern "C" void TIMER3_IRQHandler_v(void) {
  NVIC_DisableIRQ(TIMER3_IRQn);

  if (NRF_TIMER3->EVENTS_COMPARE[0] == 1) {

    NRF_TIMER3->EVENTS_COMPARE[0] = 0;  // Clear the compare event

    digitalWrite(triacPin, HIGH);  // Create a puls to trigger the triac
    delayMicroseconds(pulseWidthUs);
    digitalWrite(triacPin, LOW);

    NRF_TIMER3->TASKS_STOP = 1;  // Stop the timer
  }
  NVIC_EnableIRQ(TIMER3_IRQn);
}

#endif

void setupFanSpeed() {

#ifdef PWM

  //Set the PWM frequency to 250Hz (4ms), set duty-cycle to 0 -> Fan is off

  pwmOnPin->period_ms(4);
  pwmOnPin->write(0.0f);

#endif

#ifdef ZCD

  // configure the pin used to trigger the triac

  pinMode(triacPin, OUTPUT);
  digitalWrite(triacPin, LOW);

  // setup Timer3: 32b, timermode, 1Mhz, automatic clear

  NRF_TIMER3->BITMODE = TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos;
  NRF_TIMER3->MODE = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;
  NRF_TIMER3->PRESCALER = 4UL << TIMER_PRESCALER_PRESCALER_Pos;
  NRF_TIMER3->INTENSET = TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos;
  NRF_TIMER3->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos;

  // configure the pin used to generate an interupt on a rising edge

  pinMode(digitalPinToInterrupt(interruptPin), INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin), handleZC, RISING);

#endif
}

void adjustFanSpeed(int percentage) {

  if (percentage < 0) percentage = 0;
  if (percentage > 100) percentage = 100;

#ifdef PWM

  pwmOnPin->write(percentage / 100.0f);  //division must result in a float

#endif

#ifdef ZCD

  dimmingPercentage = percentage;

#endif
}


void setup() {

  // do needed setups for the attached fan controller/dimmer

  setupFanSpeed();

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

  while (!BLE.begin()) {  //try until you die...
    digitalWrite(LEDR, LOW);
    delay(1000);
  };
  digitalWrite(LEDR, HIGH);

#ifdef FANSTARTIMMEDIATE&& FANSSTARTSPEED
  adjustFanSpeed(FANSSTARTSPEED);
#endif

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
      delay(500);
      while (!BLE.begin()) {  //try until you die...
        digitalWrite(LEDR, LOW);
        delay(1000);
      };
      digitalWrite(LEDR, HIGH);
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
      HRsampleValue[HRsampleCount] = HRMcharvalue[1];  //retrieve the HR from the 2nd byte
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

        // adjust the speed of the fan

        adjustFanSpeed(FanDutyCycle);

        // clear the array with HR samples

        HRsampleCount = 0;
        for (int i = 0; i < HRSAMPLES; i++) HRsampleValue[i] = 0;
      }
    }

    delay(1000);
  }

  // connection lost, set the fan to it's lowest defined speed, switch of the green LED

  adjustFanSpeed(MINFAN);

  digitalWrite(LEDG, HIGH);
}
