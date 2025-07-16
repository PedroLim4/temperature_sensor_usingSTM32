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
* [Demonstration (Images/Videos)](#-demonstration-imagesvideos)
* [Challenges & Learnings](#-challenges--learnings)
* [License](#-license)
* [Contact](#-contact)

---

## ✨ Features

* Continuous reading of ambient temperature via [MENTION SENSOR MODEL, e.g., DHT11, LM35, DS18B20] sensor.
* Display of temperature on [MENTION WHERE TEMPERATURE IS DISPLAYED, e.g., an LCD 16x2 display, Serial Monitor, RGB LED].
* [ADD OTHER SPECIFIC FEATURES: e.g., Conversion between Celsius and Fahrenheit, Temperature alarm, Serial Communication with PC, etc.].

---

## 🛠️ Technologies Used

* **Microcontroller:** STM32 ([MENTION SPECIFIC STM32 MODEL, e.g., STM32F401RE, STM32L476RG]).
* **Development Board:** [MENTION THE BOARD USED, e.g., STM32 Nucleo-64, STM32 Discovery Kit, custom PCB].
* **Temperature Sensor:** [SENSOR MODEL, e.g., DHT11, LM35, DS18B20].
* **Display:** [DISPLAY TYPE, e.g., LCD 16x2, OLED, 7-segment LED - if applicable].
* **Development Environment (IDE):** [E.G., STM32CubeIDE, Keil uVision, VS Code with PlatformIO].
* **Programming Language:** C/C++.
* **Libraries/Frameworks:** [MENTION SPECIFIC LIBRARIES FROM STM32CubeMX, HAL, LL, or external libraries you used for the sensor/display].

---

## 🔌 Hardware Setup

### Circuit Diagram

[**IMAGE HERE!**]
*(If you have a Fritzing diagram, a KiCad/Altium schematic, or even a clear photo of your breadboard with connections, insert it here. If not, describe the connections in text, like below:)*

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
    * Install [YOUR IDE, e.g., STM32CubeIDE].
    * Have your [YOUR BOARD] development board and [YOUR TEMPERATURE SENSOR] ready.
2.  **Clone the Repository:**
    ```bash
    git clone [https://github.com/YourUsername/YourTemperatureSensorRepoName.git](https://github.com/YourUsername/YourTemperatureSensorRepoName.git)
    cd YourTemperatureSensorRepoName
    ```
3.  **Open Project in IDE:**
    * Open [YOUR IDE] and import the project (`.ioc` for CubeIDE or the corresponding project file).
4.  **Microcontroller Configuration (if needed):**
    * Verify pin configurations and peripherals in [YOUR IDE or STM32CubeMX].
5.  **Compile:**
    * Compile the code to generate the `.hex` or `.bin` file.
6.  **Flash:**
    * Connect your STM32 board via USB/ST-Link cable and use the IDE's flash tool to upload the firmware.

---

## 🚀 How to Use

After flashing the firmware to the STM32 board and making the hardware connections as per the diagram:

1.  Power on the STM32 board.
2.  The temperature will be displayed on the [YOUR DISPLAY/SERIAL MONITOR].
3.  [ADD ANY INTERACTION INSTRUCTIONS, e.g., Press button to change units, etc.].

---

## 📸 Demonstration (Images/Videos)

[**IMAGE HERE!**]
*(Insert a clear photo of your assembled and working prototype.)*

[**IMAGE HERE!**]
*(Insert another photo, perhaps of details or the display showing the reading.)*

[**VIDEO HERE!**]
*(If you have a short video on YouTube or Vimeo showcasing the project in action, paste the link or embed it here. This is **highly impactful** for hardware projects!)*

---

## 🚧 Challenges & Learnings

* [**CHALLENGE 1:** Describe a challenge you faced, e.g., "Difficulty in communicating with the DHT11 sensor and ensuring accurate data readings."]
    * **Learning:** "[LEARNING 1]: I overcame this by thoroughly studying the sensor's datasheet and implementing precise timers to ensure correct communication protocol timing."
* [**CHALLENGE 2:** Another challenge, e.g., "Optimizing power consumption for continuous operation."]
    * **Learning:** "[LEARNING 2]: I learned how to configure STM32's low-power modes to extend battery life."
* [ADD MORE CHALLENGES AND LEARNINGS, if any].

---

## 📄 License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for more details.

---

## ✉️ Contact

* **Pedro Victor Lima** - [My LinkedIn Profile](https://www.linkedin.com/in/yourlinkedinusername)

---
