# ENEL 300 RC Car – Embedded Systems Final Project

# RC-Nissan-GTR
RC Car modeled after the Iconic Nissan GTR r35. Designed from scratch using AVR128DB28 MCU, ultrasonic sensors, joystick control, I2C LCD, and L298N motor driver. Built in MPLAB X, with custom PCBs, bare-metal C firmware, and real-time sensor feedback.

## Overview
This project is a fully functional RC car built from scratch using embedded C on the AVR128DB28 microcontroller. It includes:

- motor control via custom built L298N motor driver PCB
- Ultrasonic sensor for obstacle detection
- 4 digit display for distance feedback
- Remote controller using joystick module (KY-023) and Wireless 433Mhz RF Transmitter/Receiver Modules
- Custom PCBs and real-time sensor integration

## Features
- Real-time distance sensing (HC-SR04)
- Metal detection with buzzer output
- Headlight control (GPIO driven)
- Directional motor control with PWM
- I2C LCD for debug/status

## Technologies
- AVR128DB28 microcontroller
- Bare-metal C (no Arduino)
- MPLAB X IDE
- I2C, PWM, ADC, GPIO
- Custom PCB Design (KiCAD)

## Documentation
See [`Documentation/`](Documentation/) folder for project reports and architecture

Read the full project report here:  
[ENEL 300 RC Car Final Report (Google Docs)]((https://docs.google.com/document/d/1p-epSyup4PSlBaB8h3Y5LkGUN4icb1V6V-HfzxVr1pI/edit?tab=t.0))

## Author
Hamza Elhakim – Electrical and Computer Engineering Student


