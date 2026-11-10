# lfr-trial-esp-1

**ESP32 PID Fast Line Follower Robot**

**Author**: Kenneth ([@kenneth3132](https://github.com/kenneth3132))

Custom high-speed PID line follower robot firmware and hardware configuration designed for:
- **Microcontroller**: ESP32-D0WD-V3 (Dual Core 240MHz, 4MB Flash)
- **Sensor Array**: SmartElex RLS-08 (8 IR Reflectance Sensors, Direct ADC connection)
- **Motor Driver**: TB6612FNG Dual H-Bridge Driver
- **Controls**: 3 Push Buttons + Built-in Status LED (GPIO 2)

---

## 1. Hardware Pinout Table

### SmartElex RLS-08 to ESP32 (Direct ADC, No MUX)
| Sensor | Function | ESP32 GPIO | Description |
| :--- | :--- | :--- | :--- |
| **S1** | Leftmost Sensor | `GPIO 36` (VP) | ADC1_CH0 |
| **S2** | Sensor 2 | `GPIO 39` (VN) | ADC1_CH3 |
| **S3** | Sensor 3 | `GPIO 34` | ADC1_CH6 |
| **S4** | Sensor 4 (Mid-Left) | `GPIO 35` | ADC1_CH7 |
| **S5** | Sensor 5 (Mid-Right) | `GPIO 32` | ADC1_CH4 |
| **S6** | Sensor 6 | `GPIO 33` | ADC1_CH5 |
| **S7** | Sensor 7 | `GPIO 25` | ADC2_CH8 |
| **S8** | Rightmost Sensor | `GPIO 26` | ADC2_CH9 |
| **VCC** | Power | `5V` (or 3.3V) | From ESP32 / Regulator |
| **GND** | Ground | `GND` | Common Ground |

### TB6612FNG Motor Driver to ESP32
| TB6612FNG Pin | ESP32 GPIO | Description |
| :--- | :--- | :--- |
| **PWMA** | `GPIO 17` | Left Motor Speed Control |
| **AIN1** | `GPIO 4` | Left Motor Direction 1 |
| **AIN2** | `GPIO 16` | Left Motor Direction 2 |
| **PWMB** | `GPIO 15` | Right Motor Speed Control |
| **BIN1** | `GPIO 5` | Right Motor Direction 1 |
| **BIN2** | `GPIO 18` | Right Motor Direction 2 |
| **STBY** | `GPIO 19` (or `3.3V`) | Standby Pin (MUST be HIGH to enable motors) |
| **VM** | `7.4V` LiPo | Motor Power Supply |
| **VCC** | `3.3V` or `5V` | Logic Power Supply |
| **GND** | `GND` | Common Ground |

### 3 Push Buttons & Status LED
| Control | ESP32 GPIO | Connection | Action |
| :--- | :--- | :--- | :--- |
| **Button 1** | `GPIO 27` | Pin to GND (Internal Pull-Up) | **START / STOP** PID line follower |
| **Button 2** | `GPIO 14` | Pin to GND (Internal Pull-Up) | **CALIBRATE** (Sweep 4s over track) |
| **Button 3** | `GPIO 13` | Pin to GND (Internal Pull-Up) | **TOGGLE** Black / White line mode |
| **Status LED** | `GPIO 2` | Built-in on ESP32 DevKit | Blinks during calibration; Solid when running |

---

## 2. Arduino IDE Setup & Upload

1. Open **Arduino IDE**.
2. Go to **Tools > Board > esp32 > ESP32 Dev Module**.
3. Select **Tools > Port > COM9** (or your ESP32 COM port).
4. Settings:
   - **Upload Speed**: `921600` (or `115200`)
   - **CPU Frequency**: `240MHz`
   - **Flash Frequency**: `80MHz`
   - **Flash Mode**: `QIO` or `DIO`
   - **Partition Scheme**: `Default 4MB with spiffs`

---

## 3. Step-by-Step Calibration & Running

### Step 1: Test & Calibrate with `sensor_reader.ino`
1. Open [`sensor_reader/sensor_reader.ino`](sensor_reader/sensor_reader.ino) in Arduino IDE.
2. Upload to ESP32.
3. Open **Serial Monitor** at **115200 baud**.
4. Press **Button 2** (GPIO 14) and slide the robot slowly left and right across the line for 4 seconds.
5. Watch the live display show:
   `[ .  .  .  #  #  .  .  . ] | Pos: +120 | Mode: BLACK`

### Step 2: Run the Competition Firmware with `lfr_esp32_pid.ino`
1. Open [`lfr_esp32_pid/lfr_esp32_pid.ino`](lfr_esp32_pid/lfr_esp32_pid.ino) (or [`lfr_esp32_modular/lfr_esp32_modular.ino`](lfr_esp32_modular/lfr_esp32_modular.ino)).
2. Upload to ESP32.
3. Place robot on track.
4. Press **Button 2** (GPIO 14) to calibrate the track surface.
5. (Optional) Press **Button 3** (GPIO 13) if running on an inverted white line track.
6. Press **Button 1** (GPIO 27) -> The LED turns solid ON and the robot starts following the line at high speed!
7. Press **Button 1** again at any time to immediately stop the motors.
