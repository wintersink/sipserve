# SipServe

## Summary

SipServe is an autonomous delivery robot that uses a range of analog and dgital sensors to reliably traverse its environment to deliver beverages to several designated "Tables".

## Hardware

### Components

- 5 Ultrasonic Senors
- RPi 5
- Oaklite Camera
- 20V Li Battery + Battery Tap
- DROK Buck-Boost Converter
- Yahboom 4-channel Motor Encoder
- Chassis
    - Tank Treads - Differential Drive
    - Plexiglass Platters
    - 3D Printed Attachments w/ M Screws/Bolts/Inserts

### Software

- Python
- VSCode
- SSH
    - us: testbot
    - hn: sipserve-johann

### Notes

smoketests/IIC.py is actually the yahboom driver tutorial, and is referenced in the other smoketests for driving the motor correctly DO NOT RENAME

### RPi 5 Pinout

- SDA = Pin 3
- SCL = Pin 5
- 3.3V Power = Pin 1
- 5V Power = Pin 2,4
- GND = Pin 6,9,14

- I2C
    - Yahboom Motor Driver
        - 5V  - RED
        - GND - BLACK
        - SCL - YELLOW
        - SDA - GREEN

### Screws

- LCD Attachment
	- M3 screws and heatset inserts maybe also with standoffs
	- M6 to attach to plate, with bolt if on plexiglass, with heatset insert if on PLA
- Ultrasonic Attachment
	- M4 screws and heatset inserts to attach components to attachment
- RPi 5
	- M2 screws