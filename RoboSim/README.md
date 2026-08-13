RoboSim SmartCell
RoboSim SmartCell is a 6-DOF industrial robot simulation developed using PyQt5 and PyVista. This project integrates robot kinematics, a teach pendant approach, factory automation, quality control, a production dashboard, a virtual PLC I/O panel, and ABB RAPID code generation into a unified desktop HMI.

🚀 Features
• 3D simulation with a 6-axis ABB IRB1600 robot model

• Forward/Inverse Kinematics (FK/IK) based manual and target coordinate control

• World jogging and joint-based manual control

• Waypoint recording, playback, and trajectory tracking

• ABB RAPID-style code generation

• Voice command support

• Optional hand tracking via a separate camera worker architecture

• Pick-and-place factory automation simulation

• Quality control and defective product rejection sorting

• SmartCell Dashboard for production analytics

• Virtual PLC I/O panel

• TXT/CSV production report exporting

🛠️ Installation
Navigate to the project directory in a Python 3.12 environment:

```

cd C:\RoboSim\robotik

pip install -r requirements.txt

```

Note: PyAudio installation may require additional compilers on some Windows systems. If voice commands are not needed, the main simulation will still run without it.

💻 Usage
```

cd C:\RoboSim\robotik

python robot_sim.py

```

Alternatively, using a specific Python interpreter:

```

& "C:/Users/Kerem Öge/AppData/Local/Programs/Python/Python312/python.exe" robot_sim.py

```

⚙️ Tech Stack
• Language: Python

• GUI: PyQt5

• 3D Visualization: PyVista / VTK, pyvistaqt

• Mathematics & Kinematics: NumPy, SciPy

• Computer Vision: OpenCV, MediaPipe

• Audio Processing: SpeechRecognition

🏗️ System Architecture
The main application runs within `robot_sim.py`. PyQt5 handles the HMI, while PyVista manages the 3D robot scene. Robot kinematics are calculated using NumPy and SciPy. To prevent DLL conflicts and camera freezing on Windows, the camera and MediaPipe hand tracking operations are isolated from the main GUI thread and executed via `hand_tracker_worker.py`.

📁 File Structure
```

RoboSim/

  ├── robot_sim.py

  ├── hand_tracker_worker.py

  ├── requirements.txt

  ├── README.md

  ├── USER_MANUAL.md

  ├── RAPOR_NOTLARI.md

  ├── reports/

  ├── ABB_IRB1600_145-*.stl

  └── robot_icon.png

```

Important: STL files must remain in the same directory as `robot_sim.py` to ensure proper loading of the robot model.

📌 Known Issues & Notes
• Hand tracking is optional; the main simulation will launch even if the camera backend fails to initialize.

• MediaPipe/OpenCV are executed via a worker process and are not imported into the main GUI process.

• Windows permissions for microphone and camera access must be enabled.

• Production reports are automatically saved in the `reports/` directory with timestamped filenames.

---

Developer: Kerem Öge