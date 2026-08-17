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
The main application runs inside robot_sim.py.PyQt5 provides the desktop HMI, while PyVista / VTK manages the 3D robot environment.Robot kinematics and numerical calculations are handled using NumPy and SciPy.Camera processing and hand tracking are isolated from the main GUI through hand_tracker_worker.py to prevent camera-related blocking and Windows DLL conflicts.Technical WorkflowsRobot KinematicsRoboSim SmartCell provides both forward and inverse kinematics functionality for the simulated 6-DOF robot.Forward Kinematics: Joint configurations are converted into the corresponding end-effector pose.Inverse Kinematics: Given a desired Cartesian target, the system calculates a suitable joint configuration while considering the robot's kinematic constraints.Factory AutomationThe SmartCell environment simulates a simplified industrial production workflow:Plaintext  Product Input
        │
        ▼
    Robot Pick
        │
        ▼
    Inspection
        │
     ┌──┴───┐
     ▼      ▼
   Pass    Fail
     │      │
     ▼      ▼
  Output  Reject
Virtual PLC I/OA virtual PLC I/O interface is integrated into the HMI to simulate industrial automation signals. This provides a software-based environment for experimenting with:Digital inputs & outputsSensor & actuator statesProduction signalsNo physical PLC hardware is required for the simulation.ABB RAPID-Style Code GenerationRoboSim converts recorded robot movements and waypoints into ABB RAPID-style program structures. This creates a workflow connecting:Simulation → Robot Teaching → Program GenerationTechnology StackComponentTechnologyProgramming LanguagePython 3.12GUI / HMIPyQt53D VisualizationPyVista / VTK, pyvistaqtNumerical ComputingNumPy, SciPyComputer VisionOpenCV, MediaPipeVoice CommandsSpeechRecognitionRobot ModelABB IRB 1600Installation & UsageRequirements:Python 3.12WindowsCamera (optional)Microphone (optional)1. Clone the repositoryBashgit clone [https://github.com/kerem-oge/RoboSim.git](https://github.com/kerem-oge/RoboSim.git)
cd RoboSim
2. Install dependenciesBashpip install -r requirements.txt
Note: PyAudio may require additional build tools on some Windows systems. Voice commands are optional and the main simulation can run without microphone functionality.3. Run the applicationBashpython robot_sim.py
Important:The ABB IRB 1600 STL files (ABB_IRB1600_145-*.stl) must remain in the same directory as robot_sim.py unless the model loading paths are modified.Project StructurePlaintextRoboSim/
│
├── robot_sim.py
├── hand_tracker_worker.py
├── requirements.txt
│
├── README.md
├── USER_MANUAL.md
├── RAPOR_NOTLARI.md
│
├── reports/
│
├── ABB_IRB1600_145-*.stl
│
└── robot_icon.png
Future DevelopmentPotential future extensions include:Real PLC communicationOPC UA integrationROS / ROS 2 integrationReal robot communicationDigital twin synchronizationAdvanced collision detectionConveyor trackingMulti-robot cell simulationDocumentationFor detailed installation, operation, and feature instructions, see: USER_MANUAL.mdAuthorKerem ÖgeMechatronics Engineering StudentRobotics · Control Systems · Industrial Automation · Digital Manufacturing
