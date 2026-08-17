# RoboSim SmartCell

**RoboSim SmartCell** is a desktop-based industrial robot simulation and virtual manufacturing cell developed with Python, PyQt5, and PyVista.

The system combines **6-DOF robot kinematics, teach pendant control, factory automation, quality control, production monitoring, virtual PLC I/O, computer vision, and ABB RAPID-style code generation** within a unified HMI environment.

The project is designed as a software-based **Smart Manufacturing Cell / Digital Twin prototype** for studying industrial robotics, automation, and human-machine interaction.

![RoboSim SmartCell HMI](robosim_HMI_screenshot.png)

---

## Key Features

### 🤖 Robot Simulation & Control
- 3D simulation of a 6-axis ABB IRB 1600 industrial robot
- Forward Kinematics (FK) and Inverse Kinematics (IK)
- Cartesian target-based robot control
- World-coordinate jogging
- Joint-based manual control with joint limit handling
- Waypoint recording, playback, and trajectory tracking

### 🏭 Smart Manufacturing Cell
- Pick-and-place factory automation simulation
- Automated product handling
- Quality control inspection
- Defective product detection and rejection
- Real-time production statistics
- Production dashboard monitoring

### 🎮 Teach Pendant & HMI
- Integrated teach pendant interface
- Manual robot jogging
- Cartesian coordinate positioning
- Waypoint management
- Virtual PLC I/O panel
- Industrial-style HMI architecture

### 💻 Robot Programming
- ABB RAPID-style program generation
- Automatic generation of robot movement commands
- Waypoint-to-code workflow

### 👁️ Computer Vision & Interaction
- Optional camera-based hand tracking using MediaPipe
- Dedicated worker architecture for camera processing
- Voice command support
- Camera processing isolated from the main GUI to prevent blocking

### 📊 Production & Reporting
- Production statistics
- Quality control statistics
- Defective product tracking
- Timestamped TXT/CSV production report generation

---

## System Architecture

The application is built around a modular desktop HMI architecture.

```text
                    ┌─────────────────────────┐
                    │     RoboSim SmartCell   │
                    │       Desktop HMI       │
                    └────────────┬────────────┘
                                 │
              ┌──────────────────┼──────────────────┐
              │                  │                  │
              ▼                  ▼                  ▼
       Robot Control        SmartCell         User Interaction
              │              Automation              │
              │                  │           ┌───────┴───────┐
              ▼                  ▼           │               │
        FK / IK Engine      Production      Voice       Hand Tracking
              │              Control       Commands          │
              ▼                  │                           ▼
       PyVista 3D Scene          ▼                    Worker Process
                         Quality Control
                              │
                              ▼
                       Virtual PLC I/O
                              │
                              ▼
                     Production Reports
