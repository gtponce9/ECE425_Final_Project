# BME280 Environmental Sensor

## Project Overview
This project implements an environmental sensing system using the **Bosch BME280** sensor.  
The BME280 is capable of measuring **temperature, pressure, and humidity**, which are read by a **TM4C123 (Tiva C) development board** and processed in firmware.

At its current stage, the project does **not function correctly**, but the system architecture, hardware connections, and a majority of the firmware implementation are complete. This repository represents a near-complete implementation intended for further debugging and refinement.

---

## Block Diagram
The following block diagram illustrates the system architecture and signal flow between components.

![Block Diagram](BME_Layout.png)


---

## Link
https://youtube.com/shorts/l66c7wv6q-M?feature=share

---

## Component IC Functionality

### Bosch BME280 Environmental Sensor
- **Function:**  
  Measures ambient **temperature, barometric pressure, and relative humidity** using integrated sensing elements.
- **Interface:**  
  Communicates with the microcontroller via digital communication (I2C or SPI, depending on configuration).
- **Role in System:**  
  Provides real-time environmental data to the TM4C123 for processing and output.

### TM4C123 (Tiva C Series) Microcontroller
- **Function:**  
  Serves as the main controller for the system.
- **Role in System:**  
  Initializes the BME280 sensor, reads sensor data, processes measurements, and handles communication with a host PC over SPI.

---

## Parts List

| Component | Quantity | Description |
|---------|----------|-------------|
| Bosch BME280 Sensor | 1 | Temperature, pressure, and humidity sensor |
| TM4C123 Development Board | 1 | Main microcontroller platform |
| Jumper Cables | 6 | Signal and power connections |
| USB Cable | 1 | Power and data communication |

---

## Pinout Configuration
The following table shows the pin connections between the **Bosch BME280 sensor** and the **TM4C123 (Tiva C) development board** using **SPI communication**.

| BME280 Pin | Tiva Pin | Function |
|-----------|----------|----------|
| VCC | 3.3V | Power |
| GND | GND | Ground |
| SCL | PA2 | SPI Clock (SCK) |
| SDA | PA5 | SPI MOSI |
| CSB | PA3 | Chip Select (Active Low) |
| SDO | PA4 | SPI MISO |

> **Note:** The BME280 is configured for SPI mode by driving the CSB pin low.

## Notes
- Code is non functional
