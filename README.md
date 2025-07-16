# # 🌡️ Temperature Sensor with STM32

## Project Overview

This project involves the development of a temperature monitoring system using an **STM32 microcontroller**. The primary objective was to apply concepts of digital and analog electronics, along with embedded systems programming, to create a functional solution for acquiring and displaying real-time temperature data, as well as alternating LEDs when the light is turned on and off.

---

## Table of Contents

* [Project Overview](#-project-overview)
* [Features](#-features)
* [Technologies Used](#-technologies-used)
* [Hardware Setup](#-hardware-setup)
* [Installation & Compilation](#-installation--compilation)
* [How to Use](#-how-to-use)
* [License](#-license)
* [Contact](#-contact)

---

## ✨ Features

* Continuous reading of ambient temperature via BMP280 sensor.
* Display of temperature on shield multi-funtion.
* LED turns on when the light is on and turns off when it is off.

---

## 🛠️ Technologies Used

* **Microcontroller:** STM32F103C8T6.
* **Temperature Sensor:** BMP280.
* **Display:** Shield multi-funtion.
* **Development Environment (IDE):** STM32CubeIDE.
* **Programming Language:** C.
* **Libraries/Frameworks:** HAL.

---

## 🔌 Hardware Setup

### Circuit Diagram

---

#### **BMP280 Sensor Connections:**

| BMP280 (sensor) | STM32 (microcontroller) |
| :-------------- | :---------------------- |
| SDA             | PB7                     |
| SCL             | PB6                     |

---

#### **LDR (Luminosity Resistor) Connections:**

| LDR (Luminosity Resistor)                   | STM32 (microcontroller) |
| :------------------------------------------ | :---------------------- |
| Between LDR and 10k Ohm Resistor (Voltage Divider) | PA0                     |

---

#### **Multi-Function Shield Connections:**

| MULTI-FUNCTION SHIELD (module) | STM32 (microcontroller) |
| :----------------------------- | :---------------------- |
| PA1                            | PA3                     |
| PD4                            | PB10                    |
| PD7                            | PB1                     |
| PD8                            | PB0                     |
| PD10                           | PC13                    |

---
* **Temperature Sensor:**
    * VCC: Connected to STM32's 3.3V.
    * GND: Connected to STM32's GND.
    * DATA/OUT: Connected to STM32 pin [MENTION STM32 PIN, e.g., PA1, PB0].
* **LCD/OLED Display (if applicable):**
    * [Describe display connections to STM32 pins, e.g., I2C pins (SDA, SCL) or GPIO pins for parallel communication].
* [ADD OTHER COMPONENTS AND THEIR CONNECTIONS, e.g., LEDs, Push Buttons].

---

## ⚙️ Installation & Compilation

To compile and upload this project to your STM32 microcontroller:

1.  **Prerequisites:**
    * Install STM32CubeIDE.
    * Have your YOUR BOARD development board and YOUR TEMPERATURE SENSOR ready.
2.  **Clone the Repository:**
    ```bash
    git clone [https://github.com/PedroLim4/temperature_sensor_usingSTM32.git](https://github.com/PedroLim4/temperature_sensor_usingSTM32.git)
    cd temperature_sensor_usingSTM32
    ```
3.  **Open Project in IDE:**
    * Open YOUR IDE and import the project (`.ioc` for CubeIDE or the corresponding project file).
4.  **Microcontroller Configuration (if needed):**
    * Verify pin configurations and peripherals in YOUR IDE or STM32CubeMX.
5.  **Compile:**
    * Compile the code to generate the `.hex` or `.bin` file.
6.  **Flash:**
    * Connect your STM32 board via USB/ST-Link cable and use the IDE's flash tool to upload the firmware.

---

## 🚀 How to Use

After flashing the firmware to the STM32 board and making the hardware connections as per the diagram:

1.  Power on the STM32 board.
2.  The temperature will be displayed on the YOUR DISPLAY.

---

## 📄 License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for more details.

---

## ✉️ Contact

* **Pedro Victor Lima** - [My LinkedIn Profile](https://www.linkedin.com/in/yourlinkedinusername)

---
