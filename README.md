# Photovoltaic System Measuring Device 

## (Note: This is a course project and most of the features and spicifications are derived from the projects specification.)

## Overview

This system is designed to measure the performance of a photovoltaic (PV) module, including various environmental parameters and module efficiency. The device uses a UART interface for communication and allows both manual and automated measurement sequences, providing real-time feedback via LEDs and an LCD display. It supports system calibration and cleanness indicators based on the efficiency of the PV panel.

![PV monitoring system](https://github.com/ButiTando/SolarProject/blob/main/PVMonitor.HEIC)

## Features

- **Power Supply**: Regulated 5V and 3.3V supplies are generated from a 9-12V input. LED D6 indicates the presence of the 5V supply.
- **PV Module Measurement**:
  - Measures voltage (VPV), current (IPV), and maximum power point (VMPP, IMPP) values.
  - Calculates power (PMPP) and efficiency (PEFF) of the PV module.
  - Supports manual and automated load adjustments for measurement.
  - Displays results on an LCD and transmits data via UART.
  - Measurement sequence can be triggered using UART commands or a bottom pushbutton.
  - LED D2 indicates when a measurement is in progress (blinks) and when it has completed (stays on).
- **Environmental Measurement**:
  - Measures ambient temperature (Ta), solar panel temperature (Tsp), and light intensity (Lxd).
  - Results are displayed on an LCD and transmitted via UART.
  - LED D3 indicates measurement status.
- **Cleanness Indicator**:
  - Provides a visual indication (LED D5) on whether the PV panel needs cleaning based on efficiency.
- **Calibration**:
  - The system can be calibrated when the panel is clean, establishing a baseline efficiency for the module.
  - Calibration is initiated via UART commands or the right pushbutton.
  - LED D4 flashes during the process and stays on when complete.
- **LCD Display**:
  - The LCD updates with real-time measurement data and cycles through various display modes without delay.
- **UART Communication**:
  - Transmits and receives commands in a specific format (115200 baud rate, 8 data bits, 1 odd parity bit, 1 stop bit).
  - Reports PV and environmental data, and sends the owner’s student number after startup.
- **Real-Time Clock (RTC)**:
  - The device includes an RTC for timekeeping.

## Getting Started

### Hardware Requirements

- 9V to 12V power supply or battery.
- 5V and 3.3V voltage regulators (5V: LM235 and 3.3V: MCP1700)
- Photovoltaic (PV) panel.
- TIP31C power transistor (Any power transistor will work. The transistor is used in an active load configuration to automatically find the maximum power point(MPP)).
- LEDs for visual feedback.
- 16x2 LCD display.
- Temperature sensors (Digital temperature sensoror: LMT01 and Analog temperature sensor: LM235).
- Photodiode for light intensity measurement (SFH203).
- STM32F411 Nucleo development board.
- Tactile switches.

### Software Requirements

- Embedded C/C++ development environment (STM32CubeIDE or Visual studio code with PlatformIO extension)
- UART communication software ( Termite on window and Minicom on Linux)

### Installation

1. Connect the power supply and the PV module to the system as per the hardware design.
2. Ensure all sensors (temperature, light intensity) and LEDs are properly connected.
3. Upload the provided firmware to the microcontroller using USB connector on the Nucleo board.
4. Initialize the UART communication at a baud rate of 115200, 8 data bits, 1 odd parity bit, and 1 stop bit.

### Usage

#### Power Supply Indicator
- When the system is powered on, **LED D6** will indicate that the 5V regulated voltage is present.

#### PV Module Measurement
1. Initiate the measurement sequence via:
   - **UART command**: Send the SP measurement command.
   - **Bottom pushbutton**: Press to start the sequence.
2. Monitor **LED D2**:
   - Blinks (100 ms on, 100 ms off) during measurement.
   - Stays on once the measurement sequence is complete.
3. View real-time results (VPV, IPV, PPV) on the LCD. After the sequence, the LCD will show VMPP, IMPP, PMPP, and PEFF values.
4. Data is transmitted via UART.

#### Environmental Measurement
1. Start the environmental measurement sequence using:
   - **UART command**: Send the EN measurement command.
   - **Top pushbutton**: Press to start the sequence.
2. Watch **LED D3** for measurement status:
   - Blinks (50 ms on, 50 ms off) during measurement.
   - Stays on after the sequence completes.
3. The LCD will display Ta, Tsp, and Lxd values, and data will be transmitted via UART.

#### Cleanness Indicator
- **LED D5** will:
  - Blink rapidly (100 ms on, 100 ms off) if cleaning is needed.
  - Stay on continuously if the panel is clean.

#### Calibration
1. Initiate calibration via:
   - **UART command**: Send the CA measurement command.
   - **Right pushbutton**: Press to start.
2. **LED D4** will blink (200 ms on, 200 ms off) during calibration and stay on after it completes.

#### UART Communication
- The system supports two-way communication via UART:
  - Send commands to initiate measurements.
  - Receive data points (VMPP, IMPP, PMPP, PEFF, Ta, Tsp, Lxd) from the device.

### Command Reference

- **SP**: Start/stop PV module measurement.
- **EN**: Start/stop environmental measurement.
- **CA**: Start calibration sequence.
- **UART Format**:
  - Command format: &_<Command>_*\n. Example of enviroment command: &_EN_*\n.

### Display Modes

The system has multiple LCD display modes, which update in real-time. In Mode 4, the LCD cycles through display options every 2 seconds.

### Maintenance

- Periodically check sensor connections and LED functionality.
- Ensure the PV panel is clean for accurate measurements.
- Perform calibration when the panel is free of dust or debris.

### Troubleshooting

- If LEDs do not behave as expected, check power supply connections.
- For UART communication issues, verify the baud rate and data format.
- If calibration seems inaccurate, ensure the panel is clean and try recalibrating.
- Ensure solder joints are clean.
