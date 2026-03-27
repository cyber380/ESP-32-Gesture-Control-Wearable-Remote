# ESP-32-Gesture-Control-Wearable-Remote
Gesture-controlled wearable remote using ESP32 and MPU6050 with OLED-based interface for real-time multi-device control.
## Description
This project is a gesture-controlled wearable remote system built using ESP32 and MPU6050. It allows a user to control external devices such as RC cars or robotic vehicles through wrist movements.

The system interprets motion data from the MPU6050 sensor and converts it into control commands like forward, backward, left, right, and stop.

## Features
- Gesture-based motion control using MPU6050
- Real-time wireless control of devices
- OLED display interface for menu navigation
- Multi-device selection capability
- Button-based control for scrolling and selection

## Components Used
- ESP32
- MPU6050 (Accelerometer + Gyroscope)
- OLED Display
- Push Buttons

## Working
The MPU6050 sensor captures wrist movements and sends motion data to the ESP32. Based on predefined thresholds, gestures are mapped to control commands. The OLED display provides a menu interface to select and control different connected devices using buttons.

## Simulation : https://wokwi.com/projects/459640346619932673
