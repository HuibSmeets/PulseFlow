# PulseFlow
Control the speed of a cooling fan with your heart rate

<img src="Prototype.JPG" alt="Alt text" width="400">
Image: Prototype, work in progress

## Description

**PulseFlow** is a small Arduino sketch that connects via BLE to a HR fitness sensor like those from Garmin, Wahoo and others. The fan is connected to a PWM controllable AC dimmer. These AC dimmers can be sourced from marketplaces like AliExpress.

## Warning
BE AWARE THAT YOU'LL BE WORKING WITH DEADLY LIVE VOLTAGES. TAKE APPROPRIATE SAFETY MEASURES WHEN CONNECTING PARTS AND WHILE USING THE DEVICE YOU CREATED. 

## Limitations

+ Your HR sensor must allow for multiple BLE connections as your trainer software/device will possiby have to connect to the sensor too using BLE (Garmin HR-Dual sensor supports multiple BLE connections)
+ Sensor pairing is hardcoded, see the sourcecode
+ Use a fan that has no electronic speed control of it's own.
+ The HR to fan speed mapping is hardcoded, see the sourcecode
+ The PWM signal is available on pin D6 (Arduino numbering)
+ The PWM signal is not generated by analogWrite as the standard PWM frequency is too high for the AC Dimmer I used
+ **The PWM related code is therefore specific for an Arduino NANO 33 BLE rev2** (but should be easy to adapt)
+ As the NANO is a 3.3V device the AC dimmer you use must be 3.3v tolerable/compatible (or add levelshifting)
+ Installing the various parts in a metal of shielded housing can influense the BLE RF signal

## Possible enhancements

+ Support different AC dimmer (not the PWM controlled one's)
+ Do away with hardcoded pairing
+ Create an UI (e.g. a 2-Line display and rotary encoder) for pairing, map settings. 
+ Alternate fan speed mapping methods (speed, power,...).
