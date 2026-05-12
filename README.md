# AUV-Phytoplankton-project

## EEE4113F Group Project — University of Cape Town

An autonomous underwater vehicle (AUV) designed for vertical water column profiling and phytoplankton concentration measurement. The vehicle uses a buoyancy-driven mechanism to control depth, making it quiet, energy-efficient, and minimally disruptive to the surrounding marine environment.

## Project overview
Phytoplankton play a critical role in marine ecosystems and are sensitive indicators of ocean health. This AUV autonomously profiles the water column, measuring phytoplankton concentration at varying depths without external control or propulsion. By using buoyancy rather than thrusters, the vehicle avoids disturbing the water column during measurement — improving data quality and reducing power consumption.

# Repository structure
```
AUV-Phytoplankton-Project/
│
├── Depth Controller/       # Control system design and firmware
├── Actuator Design/        # Buoyancy actuator design and 3D models
├── Housing Design/         # Pressure housing design and 3D models
└── Power/                  # Power electronics and electrical systems
```

# Subsystems
### 🎛️ Depth controller
The depth control subsystem is responsible for autonomously regulating the vehicle's depth during a dive profile. It processes sensor data and drives the buoyancy actuator to track target depth setpoints.

### ⚙️ Actuator design
The buoyancy actuator controls the vehicle's depth by varying its displaced volume. A syringe mechanism draws in or expels water to make the vehicle sink or rise, with 3D-printed components designed for reliable underwater operation.

### 🏠 Housing design
The pressure housing provides a watertight enclosure for all onboard electronics. It is designed to withstand hydrostatic pressure during dives while remaining compact and manufacturable using available fabrication methods.

### 🔋 Power
The power subsystem is responsible for all electrical power on the vehicle. This includes battery selection and charging, power distribution to all subsystems, motor drive circuitry, voltage regulation for the microcontroller and sensors, and the design of supporting circuits such as switching circuits and protection electronics.

## Getting started
Each subsystem folder contains its own documentation. Refer to the individual folders for design files, source code, and setup instructions specific to each subsystem.
