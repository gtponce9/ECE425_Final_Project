# BME280 Environmental Sensor

## Project Overview
This project implements an environmental sensing system using the **Bosch BME280** sensor.  
The BME280 is capable of measuring **temperature, pressure, and humidity**, which are read by a **TM4C123 (Tiva C) development board** and processed in firmware.

At its current stage, the project does **not function correctly**, but the system architecture, hardware connections, and a majority of the firmware implementation are complete. This repository represents a near-complete implementation intended for further debugging and refinement.

---

## Block Diagram
The following block diagram illustrates the system architecture and signal flow between components.

![Block Diagram](BME_Layout.png)

> Ensure `BME_Layout.png` is placed in the root of the repository or update the path if it is stored in a subfolder.

---

## Link
- GitHub Repository: https://github.com/USERNAME/REPOSITORY_NAME

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
  Initializes the BME280 sensor, reads sensor data, processes measurements, and handles communication with a host PC over USB/UART.

---

## Parts List

| Component | Quantity | Description |
|---------|----------|-------------|
| Bosch BME280 Sensor | 1 | Temperature, pressure, and humidity sensor |
| TM4C123 Development Board | 1 | Main microcontroller platform |
| Jumper Cables | 6 | Signal and power connections |
| USB Cable | 1 | Power and data communication |

---

## Notes
- The project is **incomplete** and requires additional debugging to achieve full functionality.
- Future improvements include correcting sensor communication issues and validating measurement accuracy.
