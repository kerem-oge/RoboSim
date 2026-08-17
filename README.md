# RoboSim SmartCell

**RoboSim SmartCell** is a desktop-based industrial robot simulation and virtual manufacturing cell developed with Python, PyQt5, and PyVista.

The system combines **6-DOF robot kinematics, teach pendant control, factory automation, quality control, production monitoring, virtual PLC I/O, computer vision, and ABB RAPID-style code generation** within a unified HMI environment.

The project is designed as a software-based **Smart Manufacturing Cell / Digital Twin prototype** for studying industrial robotics, automation, and human-machine interaction.

![RoboSim SmartCell HMI](robosim_HMI_screenshot.png)

---

## Key Features

### 🤖 Robot Simulation & Control
*   3D simulation of a 6-axis ABB IRB 1600 industrial robot
*   Forward Kinematics (FK) & Inverse Kinematics (IK)
*   Target coordinate-based robot control & World-coordinate jogging
*   Joint-based manual control with joint limit handling
*   Waypoint recording, playback, and trajectory tracking

### 🏭 Smart Manufacturing Cell
*   Pick-and-place factory automation simulation
*   Automated product handling & Quality control inspection
*   Defective product detection and rejection
*   Real-time production statistics and dashboard monitoring

### 🎮 Teach Pendant & HMI
*   Integrated teach pendant interface for manual robot jogging
*   Coordinate-based positioning and waypoint management
*   Virtual PLC I/O panel
*   Industrial-style HMI architecture

### 💻 Robot Programming
*   ABB RAPID-style program generation
*   Automatic generation of robot movement commands
*   Waypoint-to-code workflow

### 👁️ Computer Vision & Interaction
*   Optional camera-based hand tracking using MediaPipe
*   Separate worker architecture for camera processing to prevent GUI blocking
*   Voice command support

### 📊 Production & Reporting
*   Production and quality control statistics
*   Defective product tracking
*   Timestamped TXT/CSV production report generation

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
        FK / IK Engine      Production       Voice        Hand Tracking
              │              Control         Commands       │
              ▼                  │                           │
       PyVista 3D Scene          ▼                           ▼
                         Quality Control              Worker Process
                              │
                              ▼
                       Virtual PLC I/O
                              │
                              ▼
                     Production Reports
The main application runs inside robot_sim.py.PyQt5 provides the desktop HMI, while PyVista / VTK manages the 3D robot environment.Robot kinematics and numerical calculations are handled using NumPy and SciPy.Camera processing and hand tracking are isolated from the main GUI through hand_tracker_worker.py to prevent camera-related blocking and Windows DLL conflicts.Technical WorkflowsRobot KinematicsRoboSim SmartCell provides both forward and inverse kinematics functionality for the simulated 6-DOF robot.Forward Kinematics: Joint configurations are converted into the corresponding end-effector pose.Inverse Kinematics: Given a desired Cartesian target, the system calculates a suitable joint configuration considering kinematic constraints.Factory AutomationThe SmartCell environment simulates a simplified industrial production workflow:Product Input ➔ Robot Pick ➔ Inspection ➔ Pass (Output) or Fail (Reject)Virtual PLC I/OA virtual PLC I/O interface is integrated into the HMI to simulate industrial automation signals. This provides a software-based environment for experimenting with digital inputs/outputs, sensor states, and production signals without requiring physical hardware.ABB RAPID-Style Code GenerationRoboSim converts recorded robot movements and waypoints into ABB RAPID-style program structures, creating a bridge between Simulation ➔ Robot Teaching ➔ Program Generation.Technology StackComponentTechnologyProgramming LanguagePython 3.12GUI / HMIPyQt53D VisualizationPyVista / VTK, pyvistaqtNumerical ComputingNumPy, SciPyComputer VisionOpenCV, MediaPipeVoice CommandsSpeechRecognitionRobot ModelABB IRB 1600Installation & UsageRequirements: Python 3.12, Windows, Camera (optional), Microphone (optional).Clone the repository and navigate to the project directory:Bashgit clone [https://github.com/kerem-oge/RoboSim.git](https://github.com/kerem-oge/RoboSim.git)
cd RoboSim
Install the required dependencies:Bashpip install -r requirements.txt
Note: PyAudio may require additional build tools on Windows. Voice commands are optional.Run the main application:Bashpython robot_sim.py
Important: The ABB IRB 1600 STL files (ABB_IRB1600_145-*.stl) must remain in the same directory as robot_sim.py.Future DevelopmentPotential future extensions include:Real PLC communication & OPC UA integrationROS / ROS 2 integrationReal robot communication & Digital twin synchronizationAdvanced collision detection and conveyor trackingMulti-robot cell simulationDocumentation & AuthorFor detailed installation, operation, and feature instructions, see the USER_MANUAL.md.Author: Kerem ÖgeMechatronics Engineering StudentRobotics · Control Systems · Industrial Automation · Digital Manufacturing
