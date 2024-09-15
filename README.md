/*
################################################################################
#                         Data Logger for BMS Application                      #
################################################################################

## Overview

This project is designed to build a Data Logger for a Battery Management System 
(BMS) application using the STM32L476RG microcontroller. It continuously logs 
the voltage, current, and power from the INA226 sensor and sends the data to 
ThingSpeak via an ESP8266 Wi-Fi module. FreeRTOS is used to manage tasks.

### Key Features
- INA226 Sensor: Measures bus voltage, current, and power.
- FreeRTOS Tasks:
  - Task 1 collects voltage and current data using I2C.
  - Task 2 transmits data to ThingSpeak via ESP8266 using UART.
- ThingSpeak Integration: Real-time data logging and visualization.
- CMSIS-RTOS API: Used to manage tasks with mutex protection for data sharing.

--------------------------------------------------------------------------------

## Hardware Requirements
- STM32L476RG Nucleo Board
- INA226 Power Monitor Sensor
- ESP8266 Wi-Fi Module
- Battery System
- Jumper wires and connectors

## Software Requirements
- STM32CubeIDE with STM32CubeMX for project configuration
- FreeRTOS middleware enabled in STM32CubeMX
- ThingSpeak account for cloud data logging and visualization

--------------------------------------------------------------------------------

## Project Architecture

The system runs two tasks:
1. **I2C Task (INA226 Data Collection)**:
   - Communicates with the INA226 sensor via I2C1.
   - Reads bus voltage and current.
   - Calculates the State of Charge (SOC) of the battery.
   - Uses a mutex to update a shared data buffer.
   
2. **UART Task (Data Transmission to ThingSpeak)**:
   - Sends the collected data to ThingSpeak via ESP8266 using UART.
   - Sends data every 15 seconds (ThingSpeak's rate limit).
   - Formats data for HTTP GET requests to send to ThingSpeak.

--------------------------------------------------------------------------------

## Project Structure

- **main.c**: 
  - Contains initialization code for I2C, UART, GPIO, and FreeRTOS.
  - Manages task creation and the core application logic.
  
- **stm32l476xx_hal.c/h**: 
  - HAL libraries for STM32 peripherals (I2C, UART).
  
- **I2C and INA226 Sensor**:
  - I2C is used to communicate with the INA226 sensor.
  - Requests bus voltage and current, processes the received data.

- **UART and ESP8266**:
  - UART is used to communicate with the ESP8266.
  - Sends HTTP requests to post data to ThingSpeak.

--------------------------------------------------------------------------------


