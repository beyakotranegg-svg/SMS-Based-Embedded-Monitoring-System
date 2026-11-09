# SMS-Based Embedded Monitoring and Control System

## Project Overview

This project consists of an embedded monitoring and control system based on the ATmega328P microcontroller.

It integrates environmental sensors, RFID authentication, GSM communication, and an LCD display to enable real-time monitoring and remote interaction through SMS.

## Hardware Components

- ATmega328P: Main microcontroller
- SIM900A: GSM module for SMS communication
- MFRC522: RFID reader for access control
- DHT11: Temperature and humidity sensor
- MQ135: Air quality sensor
- PMOD ALS: Ambient light sensor
- I2C LCD: Display of measurements and system status
- Buzzer: Audible alerts

## Main Features

- Environmental data acquisition from multiple sensors
- RFID-based access control
- Remote monitoring through SMS requests
- SMS alerts for unauthorized access
- Automatic alarms when sensor thresholds are exceeded
- Real-time display of sensor measurements on LCD

## Communication Protocols

- UART: Communication with the SIM900A GSM module
- SPI: Communication with RFID and ambient light sensors
- I2C: Communication with the LCD
- ADC: Analog sensor data acquisition

## Project Structure

### Main Application

- `code_final_V2.c`: Final application integrating sensors, RFID authentication, GSM communication, LCD display, and alarm management.

### Test Programs

- `test_RFID.c`: RFID reader testing
- `testGSM.c`: GSM and SMS communication testing
- `testDHT11.c`: Temperature and humidity sensor testing
- `testALS.c`: Ambient light sensor testing
- `testCAPTEUR.c`: Sensor integration testing
- `testAM2320.c`: AM2320 sensor testing
- `ALSBP.c`: Ambient light sensor development and testing
- `AM2320BP.c`: AM2320 sensor development and testing

## Development Tools

- Programming Language: Embedded C
- IDE: MPLAB X
- Compiler: AVR-GCC
- Microcontroller: ATmega328P (16 MHz)

## Skills Developed

- Embedded C programming
- Microcontroller programming
- Sensor interfacing
- UART, SPI, and I2C communication
- GSM communication using AT commands
- RFID authentication
- Hardware/software integration
- Embedded system debugging

## Author

Beya Kotrane

Electrical Engineering Student - ENIT

Academic Exchange - ENSTA Bretagne (SOIA)

