# ARCOproyecto - ESP32 Brushless Fan Control System

[![PlatformIO](https://img.shields.io/badge/PlatformIO-Compatible-brightgreen.svg)](https://platformio.org/)
[![ESP32](https://img.shields.io/badge/ESP32-Wemos%20D1%20Uno32-blue.svg)](https://www.espressif.com/en/products/socs/esp32)

A comprehensive ESP32-based motor control system for brushless fans with dual communication interfaces, automatic cycling, and real-time status feedback through LED indicators.

## 🚀 Features

### Core Functionality

- **Smart Motor Control**: Start/stop with speed memory retention
- **Variable Speed Control**: 31-level PWM speed control (0-255 mapped range)
- **Automatic Cycling**: Progressive speed ramping with timed sequences
- **Safety Validations**: Comprehensive error handling and state management

### Communication

- **Dual UART Interface**: USB Serial (38400 baud) + HC-05 Bluetooth module
- **Command Menu System**: Character-based control commands
- **Real-time Feedback**: Status messages and error reporting

### Visual Indicators

- **LED1**: Motor status (ON/OFF indication with blinking when stopped)
- **LED2**: Speed increment feedback
- **LED3**: Maximum RPM reached indicator
- **LED4**: Speed decrement feedback
- **Built-in LED**: Communication activity indicator

## 🛠️ Hardware Architecture

### Components

| Component | Pin | Description |
|-----------|-----|-------------|
| **ESP32 Wemos D1 Uno32** | - | Main microcontroller |
| **12V Brushless Fan** | GPIO 4 | PWM-controlled motor |
| **HC-05 Bluetooth** | RX: GPIO 16, TX: GPIO 17, KEY: GPIO 26 | Wireless communication |
| **Status LEDs** | GPIO 18, 19, 23, 5 | Visual feedback system |
| **Protection Diode** | - | Motor back-EMF protection |

### Pin Configuration

```cpp
#define RX 16          // HC-05 RX
#define TX 17          // HC-05 TX  
#define KEY 26         // HC-05 AT mode
#define LED1 18        // Motor status
#define LED2 19        // Speed up indicator
#define LED3 23        // Max speed indicator
#define LED4 5         // Speed down indicator
#define MOTOR_VENTILADOR 4  // PWM motor control
```

## 🔧 Software Architecture

### Project Structure

```text
ARCOproyecto/
├── src/
│   └── main.cpp           # Main application logic
├── include/               # Header files
├── lib/                   # Custom libraries
├── test/                  # Unit tests
├── platformio.ini         # PlatformIO configuration
└── README.MD             # This file
```

### Core Functions

- `arrancarMotor()` - Motor startup with speed restoration
- `detenerMotor()` - Safe motor shutdown with speed memory
- `incrementarVelocidad()` - Speed increment with bounds checking
- `reducirVelocidad()` - Speed decrement with bounds checking
- `cicloAutomatico()` - Automated speed cycling sequence
- `menu(byte, byte)` - Command processing for dual interfaces

### Command Interface

| Command | Action | Description |
|---------|--------|-------------|
| `P` | Stop | Safely stop motor and save current speed |
| `A` | Start | Start motor (idle or resume previous speed) |
| `I` | Increment | Increase speed by 1 level |
| `R` | Reduce | Decrease speed by 1 level |
| `C` | Cycle | Run automatic speed cycling sequence |

## ⚡ Quick Start

### Prerequisites

- [PlatformIO IDE](https://platformio.org/platformio-ide) or [PlatformIO Core](https://platformio.org/install/cli)
- ESP32 development board (Wemos D1 Uno32)
- Arduino framework knowledge

### Installation

1. **Clone the repository**

   ```bash
   git clone https://github.com/PostboxRetinal/ARCOproyecto.git
   cd ARCOproyecto
   ```

2. **Open in PlatformIO**

   ```bash
   pio run
   ```

3. **Upload to ESP32**

   ```bash
   pio run --target upload
   ```

4. **Monitor Serial Output**

   ```bash
   pio device monitor
   ```

### Wiring Diagram

```text
ESP32 Wemos D1 Uno32    HC-05 Bluetooth    12V Fan    LEDs
     GPIO 4      ------------------- PWM Input
     GPIO 16     --------- RX
     GPIO 17     --------- TX  
     GPIO 26     --------- KEY
     GPIO 18     ----------------------------- LED1 (Motor Status)
     GPIO 19     ----------------------------- LED2 (Speed Up)
     GPIO 23     ----------------------------- LED3 (Max Speed)
     GPIO 5      ----------------------------- LED4 (Speed Down)
     3V3         --------- VCC
     GND         --------- GND
```

## 🔄 Usage Examples

### Serial Control

```text
Send 'A' to start motor
Send 'I' to increase speed  
Send 'R' to reduce speed
Send 'C' to run automatic cycle
Send 'P' to stop motor
```

### Automatic Cycle Sequence

1. **Ramp Up**: 0 → 31 levels over 20 seconds
2. **Hold**: Maximum speed for 10 seconds with LED3 blinking
3. **Ramp Down**: 31 → 0 levels over 30 seconds

## 🐛 Known Issues

- **Bluetooth Communication**: HC-05 AT commands work correctly, but data transmission may be unreliable
- **Alternative Module**: HC-06 tested but AT commands not responding
