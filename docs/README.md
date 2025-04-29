# Documentation for MotorcycleSpeedCalibrator

This folder contains detailed documentation for the **MotorcycleSpeedCalibrator** project. Below is an overview of the project's purpose, circuit design, and key calculations.

---

## Project Overview

The **MotorcycleSpeedCalibrator** is an Arduino-based project designed to modify and calibrate square wave signals from a motorcycle's speed sensor. It ensures accurate speedometer readings after modifications like sprocket or tire size changes.

### Signal Flow
1. **Vehicle Sensor**: Outputs a 5V square wave signal.
2. **PC901V Input**: Opto-isolated input circuit for safe signal processing.
3. **Arduino**: Processes the signal and outputs a calibrated square wave.

---

## Circuit Design

### Protoboard Layout
![Protoboard Layout](images/protoboard_layout.jpg)


### Input Circuit (Sensor Side - 5V System)
- **+5V (from sensor system)**  
  - Connected via a 330Ω resistor (R1) to Pin 1 (Anode) of the PC901V opto-isolator.
  - Pin 2 (Cathode) is connected to GND (sensor side).

### Arduino Side (Isolated Logic Side)
- **Pin 6 (Vcc)** → +5V (Arduino)
- **Pin 5 (GND)** → GND (Arduino)  
  - Includes a 0.1 µF ceramic capacitor (C1) for decoupling.
- **Pin 4 (Vo)** → Arduino input pin (e.g., D2)  
  - Includes a 10kΩ pull-up resistor (R2) to 5V.

### Optional Noise Filter
- A 100nF ceramic capacitor (C2) can be connected across Pin 1 ↔ Pin 2 of the PC901V to reduce noise.

### Output Circuit (Optional Opto-Isolator for Output Control)
- **Arduino Output Pin (e.g., D3)**  
  - Connected via a 330Ω resistor (R3) to Pin 1 (Anode) of a second PC901V opto-isolator.
  - Pin 2 (Cathode) is connected to GND (Arduino).
- **Vehicle Side (Isolated Output)**  
  - Pin 6 (Vcc) → +5V (vehicle system)
  - Pin 5 (GND) → GND (vehicle system)  
    - Includes a 0.1 µF ceramic capacitor (C3) for decoupling.
  - Pin 4 (Vo) → Vehicle input signal wire  
    - Includes a 10kΩ pull-up resistor (R4) to 5V.

### PCB Topside
![PCB Topside](images/pcb_topside.jpg)

### PCB Underside
![PCB Underside](images/pcb_underside.jpg)

---

## Key Calculations

### Serial Communication
![Serial Communication](images/serial_communication.jpg)

### Measurement Setup
![Measurement Setup](images/measurement.jpg)

### Example Calculation
For a motorcycle with an 18-inch rear wheel:
- Rear wheel circumference:  
  `pi * 18 inches = 56.5486672 inches = 1.437 meters`
- Speed:  
  `250 km/h = 250,000 m / 3600 s = 69.444 m/s`
- Frequency:  
  `69.444 m/s ÷ 1.437 m = 48.4 Hz`
- Adjusted frequency after sprocket ratio:  
  `48.4 Hz ÷ 0.37 = 130.1 Hz`
- Final frequency (4 pulses per revolution):  
  `130.1 Hz * 4 = 520.4 Hz`
- Period:  
  `1 ÷ 520.4 Hz = 0.00192 s = 1920 µs`

---

## Pin Setup

### Arduino Pin Mapping
- **PD0–PD7**: Pins 0–7
- **PB0–PB5**: Pins 8–13

### Project-Specific Pins
- **Input Pin**: `D2` (connected to the speed sensor via opto-isolator)
- **Output Pin**: `D9` (outputs the calibrated square wave signal)
- **Built-in LED**: `D13` (used for status indication)

---

## Additional Notes

### Constants
- **Debounce Time**: `400 µs` (2500 Hz)
- **Minimum Period**: `1800 µs` (555 Hz)
- **Maximum Period**: `330,000 µs` (3 Hz)
- **Timeout Period**: Equal to `MAX_PERIOD`

### Smoothing Factor
- **Default**: `5`
- **Range**: `2–20`
- **Adaptive Option**: Uncomment `#define USE_ALPHA_ADAPTIVE` in the code to enable dynamic smoothing.

---

For further details, refer to the comments in the `main.cpp` file or the [GitHub repository](https://github.com/varsa88/MotorcycleSpeedCalibrator).