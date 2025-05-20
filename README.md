# Magic Wand Gesture Recognition

## Project Overview

This repository hosts a “magic wand” gesture recognition prototype built with a XIAO ESP32‑C3 microcontroller and an MPU‑6050 inertial sensor. The wand reads 3‑axis accelerometer data to detect three VOZ‑shaped gestures (“V”, “O”, “Z”) and provides real‑time feedback via an RGB LED. All model training and testing were performed on the Edge Impulse platform, and the quantized Int8 model is deployed in `wand.ino` for on‑device inference under constrained hardware resources.

## Hardware Components

* **XIAO ESP32‑C3** development board
* **MPU‑6050** accelerometer and gyroscope module
* **Common‑cathode RGB LED** ×1
* **Pushbutton** ×1
* **3.7 V LiPo battery pack** ×1
* Jumper wires and breadboard or custom PCB
* Enclosure (optional)

## Software Dependencies

* **Arduino IDE** (or PlatformIO)

  * Adafruit\_MPU6050
  * Adafruit\_Sensor
  * Wire
  * Edge Impulse SDK (Int8 quantized headers)
* **Python 3.8+**

  * pyserial
  * argparse

## Wiring Instructions

Ensure all ground (GND) connections share a common rail.

1. **Power**: Connect the battery’s positive terminal to XIAO VBAT, and the negative terminal to GND.
2. **MPU‑6050**: VCC → 3.3 V, GND → GND, SDA → IO21, SCL → IO22.
3. **RGB LED**: Common cathode → GND; red anode → IO7 (via 220 Ω resistor); green anode → IO8 (via 220 Ω resistor); blue anode → IO9 (via 220 Ω resistor).
4. **Pushbutton**: One leg → IO3, the other leg → GND; enable the internal pull‑up resistor on IO3 (INPUT\_PULLUP).

## Usage Steps

1. **Data Collection**

   * Upload `gesture_capture.ino` to the XIAO ESP32‑C3.
   * Trigger a 1 s capture by sending `o` over serial or pressing the button; the capture automatically stops after one second.
   * Run `process_gesture_data.py` to listen for the start marker and save each capture to a timestamped CSV file.

2. **Model Training**

   * Log into Edge Impulse and create a new impulse:

     * DSP block: 1 s window, 50 % overlap
     * ML block: two dense layers (32 and 16 neurons) with Softmax output
   * Import and label VOZ gesture CSV data.
   * Train for 30 epochs with a learning rate of 0.005 and batch size of 16.
   * Export the quantized Int8 C++ library.

3. **Deployment & Inference**

   * Copy the exported headers into the project folder.
   * Upload `wand.ino` to the XIAO ESP32‑C3.
   * Press the button to capture a gesture and run real‑time inference; the LED will illuminate according to the recognized gesture and turn off after 3 s.

## Repository Structure

```
├── README.md                    # Project overview and instructions
├── wand.ino                     # Final inference sketch
├── gesture_capture.ino          # Data collection sketch
├── process_gesture_data.py      # Serial‑to‑CSV converter script
├── edgeimpulse/                 # Exported model headers
├── data/                        # Raw CSV data
├── docs/                        # Lab report and design notes
└── enclosure/                   # Enclosure design files
```

## License

This project is released under the MIT License.

## Author

Suzy Liu (Suzy Liu)
