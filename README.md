# Assistive Grasping Glove — ESI Group 5

An engineering project exploring a motor-driven glove to assist grasping. The system uses flex sensors to detect finger bending and a force-sensitive resistor (FSR) to detect object contact.

This repository contains the control firmware, PCB design files, and 3D-printable components.

## How It Works

The controller follows a basic grasp-and-release sequence:

1. **Idle:** Wait for both flex sensors to detect finger bending.
2. **Closing:** Drive the stepper motor to assist grasping.
3. **Hold:** Stop closing when the FSR detects object contact.
4. **Opening:** Reverse the motor when the release button is pressed.
5. **Return to idle:** Wait for the next grasp.

The firmware also includes controls for flex-sensor calibration and motor homing. Their behaviour depends on the firmware version.

## Hardware

- Two flex sensors
- One force-sensitive resistor (FSR)
- Stepper motor with a DRV8825 driver
- Arduino-compatible controller
- Release, reset/home, and calibration buttons
- Custom PCB
- 3D-printed mechanical components

## Repository Contents

| File or folder | Description |
|---|---|
| [`code.c++`](code.c++) | Glove control firmware |
| [`pcb/`](pcb/) | KiCad schematic, PCB layout, and project configuration |
| [`3d-print/`](3d-print/) | Printable models and available mechanical design files |

### PCB Files

Open `project.kicad_pro` in KiCad to access the PCB project.

- `project.kicad_sch` — circuit schematic
- `project.kicad_pcb` — PCB layout and routing
- `project.kicad_pro` — project configuration

### 3D Printing Files

The `3d-print/` folder contains the mechanical files provided for the project. Check model dimensions, orientation, and fit before printing.

## Firmware Pin Assignments

| Component | Pin |
|---|---|
| Flex sensor 1 | A0 |
| Flex sensor 2 | A1 |
| FSR | A2 |
| Release button | D3 |
| Reset/home button | D4 |
| Flex calibration button | D5 |
| DRV8825 STEP | D8 |
| DRV8825 DIR | D9 |
| DRV8825 ENABLE | D10 |

## Getting Started

1. Download or clone this repository.
2. Open the PCB project in KiCad and review the wiring.
3. Create an Arduino sketch and copy the selected firmware into it.
4. Select the board and port that match the controller.
5. Check pin assignments, sensor readings, motor direction, and thresholds against the assembled hardware.
6. Upload the firmware and test calibration, grasping, holding, and release.

Sensor thresholds and motor settings depend on the physical assembly and should be checked whenever the hardware or firmware changes.

## Project Team

Developed by **ESI Group 5**.

Team members and their responsibilities can be listed here as the project documentation is completed.