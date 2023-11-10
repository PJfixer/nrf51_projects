# Bluetooth Low Energy (BLE) Motion Detection System

## Overview

This project is built on the nRF51822 platform and implements a Bluetooth Low Energy (BLE) motion detection system. The system operates primarily in sleep mode to conserve energy but can be awakened when a Passive Infrared (PIR) motion sensor detects movement. Upon detection, the system enters an advertising mode for 30 seconds, broadcasting an alarm signal over BLE.

## Hardware Components

- **nRF51822:** Main microcontroller responsible for BLE communication and system control.
- **PIR Motion Sensor:** Triggers system wake-up when motion is detected.
- **Other Components:** The project may involve additional components like resistors, capacitors, etc., as per the specific hardware design.

## Software Components

- **SoftDevice:** Manages the BLE protocol stack.
- **app_timer:** Handles timers and time-related functions.
- **ble_nam:** Custom BLE service for managing device information.
- **adc:** Configures and reads values from the Analog-to-Digital Converter (ADC).
- **flash:** Manages data storage and retrieval in flash memory.

## Operation

1. **Sleep Mode:** The system is in a low-power sleep mode to conserve energy.
2. **Motion Detection:** The PIR motion sensor detects movement and signals the nRF51822 by setting a specific pin to logic high.
3. **System Wake-up:** The rise in pin voltage triggers the nRF51822 to wake up from sleep mode.
4. **Advertising Mode:** Upon wake-up, the system enters advertising mode for 30 seconds, broadcasting an alarm signal over BLE.
5. **Data Transmission:** During the advertising period, the nRF51822 can establish BLE connections with compatible devices to transmit motion detection information.
6. **Return to Sleep:** After 30 seconds, the system returns to sleep mode until the next motion detection event.

## Configuration

### Device Name

The default device name is "Pjfixer_DETEC1". Optionally, a custom name can be stored in flash memory. Refer to the [flash.c](link_to_flash.c) file for details.

### Advertising Data

The advertising data includes device-specific information such as the manufacturer identifier and device type. Refer to [main.c](link_to_main.c) for details.

## Build and Flash

1. Clone the repository.
2. Open the project in the [nRF5 SDK](link_to_nRF5_SDK) or your preferred IDE.
3. Configure the project settings and peripherals as needed.
4. Build and flash the firmware to the nRF51822 device.

## Dependencies

- nRF5 SDK
- SoftDevice
- BLE libraries

