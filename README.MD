# ESP32 brushless fan control system

This project is a motor control system developed using the Arduino framework on a Wemos D1 Uno32 board. The system allows for controlling the speed of a motor, starting and stopping the motor, and running an automatic cycle. The system also provides feedback through serial communication and LEDs.

## Features
- Logical validations for any kind of trouble
- Start and stop the motor
- Increment and decrement motor speed
- Automatic cycle to gradually increase and decrease motor speed
- Serial communication for control and feedback (UART0 and UART2)
- LED indicators for motor status and speed changes

## Bugs
HC-05 AT commands work as expected, nonetheless, bluetooth communication doesn't seem to come through. <br> 
- (Also tested using HC06 module and didn't even answered AT calls)

## Hardware Requirements

- (ESP32) Wemos D1 Uno32 board
- 12v DC brushless fan
- LEDs (4)
- HC-05 Bluetooth module
- Diode

## Software Requirements

- PlatformIO IDE
- Arduino framework

## Installation

1. Clone this repository:
   ```sh
   git clone https://github.com/yourusername/motor-control-system.git
2. CHeck the code and wire it up as needed. If you wanna uSe another port is up to you. 
3. Upload it to your ESP32 board