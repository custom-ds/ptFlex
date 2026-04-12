# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ptFlex is an Arduino-based C++ firmware and KiCAD hardware design for a high-altitude balloon (HAB) APRS tracker. It transmits GPS position and telemetry via AX.25/APRS over VHF radio using an SA818 transceiver module on an ATmega328P microcontroller.

## Repository Layout

- `firmware/ptFlex/` — Main tracker firmware (primary development target)
- `firmware/818v-tester/` — Standalone SA818 transmitter test sketch
- `firmware/set-fuses.bat` — AVR fuse programmer (run before first flash)
- `ptFlex/` — KiCAD hardware design (schematic + PCB)
- `assets/` — Documentation images

## Building and Flashing

There is no Makefile. Build and upload via **Arduino IDE** or **PlatformIO**.

- Open `firmware/ptFlex/ptFlex.ino` as the project root.
- Target board: ATmega328P at 16 MHz.
- Build artifacts land in `firmware/ptFlex/build/`.
- Before first flash, run `firmware/set-fuses.bat` to program AVR fuses (Low: 0xDF, High: 0xD6, Extended: 0xFD).

## Firmware Architecture

The four core modules and their responsibilities:

| Module | Files | Role |
|--------|-------|------|
| **GPS** | `GPS.cpp/h` | NMEA sentence parser (GGA/RMC), GPS power control, APRS frequency selection by region |
| **Modem** | `Modem.cpp/h` | SA818 control, AX.25/APRS packet encoding, bit-stuffing, interrupt-driven NCO tone generation (1200/2200 Hz FSK via Timer1 ISR) |
| **ptConfig** | `ptConfig.cpp/h` | EEPROM-backed configuration (callsign, SSID, beacon intervals, frequencies, telemetry options) |
| **ptTracker** | `ptTracker.cpp/h` | Battery voltage monitoring, LED/buzzer/Morse output, board-level hardware interface |

`ptFlex.ino` is the main entry point. Its `loop()` drives the five beacon strategies: fixed-time, speed-based (SmartBeaconing), altitude-tier, time-slotting, and voltage-constrained (solar mode). Burst detection triggers when altitude drops >250 m from the recorded peak.

`BoardDef.h` selects the hardware variant (ptFlex vs. ptSolar) and must be set correctly before building.

`SparkFunBME280.cpp/h` is a vendored driver for the I2C pressure/temperature sensor.

## Configuration Mode (Interactive Diagnostics)

Connect via serial at 19200 baud and enter config mode. Key commands:

- `D` — Reset to factory defaults
- `E` — Exercise mode (test all I/O)
- `L` — Long transmitter test (1.5 s – 60 s)
- `P` — Send a test APRS packet over the air
- `T` — Transmitter diagnostics
- `R` / `W` — Read / write EEPROM configuration

## Key Technical Details

- **Modulation**: Timer1 Compare-Match ISR drives phase accumulation (NCO) for precise FSK tones; do not block interrupts during transmission.
- **AX.25**: Bit-stuffing is handled in `Modem.cpp`; CRC is appended automatically.
- **EEPROM layout**: Defined in `ptConfig.h`; changing field order or sizes requires a corresponding default-reset on existing hardware.
- **Battery thresholds**: Configurable minimum voltages gate GPS power and transmission separately — important for cold/solar operation.
- **Serial baud rate**: 19200 throughout (GPS input and config serial port).
