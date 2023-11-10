# nRF51822 Temperature, Humidity, and Pressure Sensor Project

This project utilizes the nRF51822 chip to create a sensor for measuring temperature, humidity, and pressure, capable of broadcasting this data via Bluetooth Low Energy (BLE). The project is designed to be ultra-low power.

## Features

- Measurement of temperature, humidity, and pressure using the BME280 sensor.
- Transmission of measured data via BLE using BLE advertisements.
- Power management for minimal energy consumption.
- Storage of the device name in flash for easy customization.


# Files generated by build
_build/

### Device Configuration

The device is configured with the default name "Pjfixer_WEA1". You can customize the name by writing to the device's flash memory.

## Source Code

The `main.c` file contains the main source code of the project, with comments explaining different parts of the code. The code uses the Nordic SoftDevice library to manage the BLE stack.

## Dependencies

- Nordic SoftDevice S130: [Download Link](https://www.nordicsemi.com/Software-and-Tools/Software/S130)
- BME280 Library: [BME280.c Source Code](https://github.com/PJfixer/PIC16F_WHEATHER_STATION/blob/master/16F18325_NO_MCC.X/bme280.c)

## Usage Instructions

1. Clone the repository.
2. Set up the development environment with the necessary dependencies.
3. Customize the device name if needed.
4. Compile and upload the code to your nRF51822 device.

## Notes

Make sure to correctly set up your development environment with the necessary tools for nRF51822 platform development.

---
