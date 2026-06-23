# Smart Wearable Compliance Monitoring System for Spinal Brace

An IoT-enabled medical electronics prototype designed to monitor patient compliance and posture while wearing a spinal brace. Driven by an ESP32, this system utilizes a network of distributed sensors to provide real-time feedback, local data logging, and a live web-based dashboard for patient monitoring.

## System Architecture

The core of the system is built around the ESP32 microcontroller, which manages an asynchronous web server and a complex I2C sensor network.

### Key Features
* **Hardware I2C Isolation:** Utilizes a TCA9548A I2C Multiplexer to isolate the RTC, OLED, and three identical MPU6050 accelerometers (sharing the `0x68` address) into distinct hardware channels to prevent bus collisions.
* **Custom Firmware Bypass:** Includes a locally modified Adafruit MPU6050 library to bypass strict `WHO_AM_I` register checks, allowing seamless integration of MPU-6500 silicon clones reporting ID `0x70`.
* **Asynchronous Web Dashboard:** Serves a live HTML/AJAX dashboard over a local WiFi network, dynamically updating sensor values, FSR ADCs, and compliance status without blocking the main hardware control loop.
* **Offline Redundancy:** Automatically falls back to a non-blocking offline mode with full functionality if the primary WiFi network (`CONRAD10`) is unavailable.
* **Data Integrity & Logging:** Synchronizes a DS3231 RTC module with SD card logging to securely record timestamped compliance and telemetry data to a local `datalog.csv` file.
* **Physical Feedback:** Triggers a localized vibration motor and LED alert when poor posture or brace removal is detected based on configurable gravitational tolerances.

## Hardware Components
* ESP32-C3 Microcontroller
* 3x MPU6050 (or MPU6500 clones) 6-DoF Accelerometer/Gyroscope
* 2x Force Sensitive Resistors (FSRs)
* TCA9548A (HW-617) I2C Multiplexer
* DS3231 Real-Time Clock (RTC)
* SSD1306 128x64 OLED Display
* MicroSD Card Module (SPI)
* Vibration Motor & Status LED

## Pin Configuration (ESP32)
| Component | Pin | Description |
| :--- | :--- | :--- |
| **I2C Bus** | GPIO 8 (SDA), GPIO 9 (SCL) | Main bus to TCA9548A Multiplexer |
| **SPI Bus (SD)** | GPIO 4 (SCK), 5 (MISO), 6 (MOSI), 7 (CS) | SD Card Module Logging |
| **FSR Sensors** | GPIO 0, GPIO 1 | Analog inputs for brace tension |
| **Actuators** | GPIO 2 (Motor), GPIO 3 (LED) | Physical compliance feedback |

## Installation & Setup

1.  **Clone the repository:**
    ```bash
    git clone [https://github.com/YourUsername/smart-spinal-brace-monitor.git](https://github.com/YourUsername/smart-spinal-brace-monitor.git)
    ```
2.  **Local Library Requirement:** Ensure the included local files (`Adafruit_MPU6050.h` and `Adafruit_MPU6050.cpp`) remain in the same root directory as the main `.ino` sketch. These contain the necessary `0x70` hardware bypass.
3.  **Dependencies:**
    Install the following libraries via the Arduino Library Manager:
    * `Adafruit GFX Library`
    * `Adafruit SSD1306`
    * `RTClib`
4.  **Network Configuration:**
    Update the `ssid` and `password` variables in the main sketch to match your local network for dashboard access.
5.  **Compile & Upload:**
    Flash the firmware to the ESP32. Upon booting, the OLED will display the local IP address assigned to the web dashboard.

---
**Author:** Awele Bonaventure Ugbah
*Chief Technology Officer, Conrad Robotics*
# Smart-Energy-Audit-ESP32
