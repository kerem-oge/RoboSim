RoboSim SmartCell User Manual
Starting the Program
Open a terminal in the project folder and run the following command:

```

python robot_sim.py

```

When the application opens, the 3D robot scene is displayed in the center area, manual control elements on the left panel, and production/automation panels on the right panel.

Manual Joint Control
The J1-J6 axes are moved individually using the joint sliders or number boxes on the left panel. Each change updates the forward kinematic position of the robot.

World Jogging
World jogging buttons move the TCP in small increments along the X, Y, and Z axes. This control utilizes the inverse kinematics solution to move the robot closer to the target TCP coordinate.

IK Target Coordinate Usage
Coordinates are entered in millimeters into the target X, Y, and Z fields. When IK is executed, the robot calculates the appropriate joint angles for this target and moves accordingly.

Waypoint Recording and Playback
The current robot position can be saved as a waypoint. The saved points are displayed in a list. The playback command moves the robot sequentially through these recorded points.

RAPID Code Generation
Once the waypoint list is prepared, an ABB RAPID-style motion program is generated using the generate code button. Target positions and motion commands are displayed in the code window.

Factory Automation
When factory automation is initiated, boxes arrive via the input conveyor. The robot picks up the box and transports it to the appropriate output based on its quality/type status. The stop button terminates the scenario and safely shuts down the automation resources.

Dashboard Overview
The SmartCell Dashboard displays robot status, active state, vacuum status, box type, total products, small/large product counts, intact/defective product counts, quality rate, cycle times, and estimated hourly production.

Quality Control Logic
Products are simulated as small intact, large intact, or defective. A small intact product is routed to the left output conveyor, while a large intact product goes to the right output conveyor. Defective products are dropped into the scrap/reject area via a safe reject route.

PLC I/O Panel
The virtual PLC I/O panel displays digital inputs and outputs as 0/1. Green indicators represent an active signal, while gray indicators represent a passive signal. This panel is strictly for simulation purposes and does not directly alter the robot's internal motion logic.

Exporting Production Reports
The production report button saves the dashboard data as TXT and CSV files into the `reports/` folder. The filenames include date and time stamps.

Voice Commands
When the voice command system is activated, the TCP can be moved using Turkish voice commands. The basic commands are:

• ileri (forward)

• geri (backward)

• sağ (right)

• sol (left)

• yukarı (up)

• aşağı (down)

If there is no microphone or internet access, the voice command module may throw an error message; however, the main simulation will continue to run seamlessly.