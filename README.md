This repository contains the complete implementation of a Multi-Drone Mission Execution & Spatio-Temporal Deconfliction System, submitted as part of the FlytBase assignment.

It includes:

Autonomous mission execution for multiple drones

Real-time trajectory recording

Spatio-temporal conflict detection

Visualizations (3D + 4D)

Documentation & final report

🧭 1. Project Overview

The goal of this assignment is to design a system that:

Executes a primary drone mission using time-tagged waypoints

Simulates other background drones whose trajectories may conflict

Performs a spatial + temporal safety check

Produces:

A “clear”/“conflict detected” status

Detailed conflict explanation

Generates:

Plots (3D, 4D)

A final report

A demonstration video

This repository contains an end-to-end, working implementation of all required components.

📁 2. Repository Contents
File	Purpose
assignment.py	Main executable pipeline (mission execution → recording → analysis → plots)
multi_mission.json	Waypoint-based mission for 3 drones
all_trajs.csv	Automatically generated — executed drone trajectories
deconflict_report.txt	Summary of detected conflicts & distances
primary_4d.png	4D visualization of primary drone trajectory
FlytBase_assignment.pdf	Final report (architecture, algorithms, scalability)
README.md	Project documentation (this file)
🛠 3. System Architecture

The system is built in five major modules:

1️⃣ Mission Loader

Loads JSON mission describing:

Drone list

Waypoints (x, y, z, t)

Takeoff altitudes

Optional staggered start

2️⃣ Drone Controller

Each drone runs in a separate thread:

Takeoff

Time-synchronized waypoint following

Velocity-based proportional control

Coordinated landing

3️⃣ Recorder

Central data logger capturing:

/ardrone_x/gt_pose

/gazebo/model_states (fallback)

Writes all_trajs.csv

This ensures clean and continuous trajectory capture.

4️⃣ Spatio-Temporal Deconfliction Engine

Two separate checks:

✔ Planned Deconfliction

Interpolates mission JSON trajectories.

✔ Executed Deconfliction

Interpolates real recorded CSV trajectories.

For each timestep:

if distance(primary, drone_i) < SAFETY_THRESHOLD:
    mark conflict


Output:

conflicts_planned.json

conflicts_executed.json

deconflict_report.txt

5️⃣ Visualization Module

Generates:

🎨 Primary Drone 4D Plot

X, Y, Z with color = time

Saved to primary_4d.png

🎨 All-Drone 3D Plot with Conflicts

Planned + executed paths

Conflict points highlighted

🚀 4. How to Run
1. Start the Gazebo Simulation
roslaunch ardrone_gazebo swarm.launch

2. Execute the Full Pipeline
python3 assignment.py


The script automatically:
✔ Runs all drones
✔ Logs trajectories
✔ Detects conflicts
✔ Generates plots
✔ Writes reports

🗺 5. Mission File Format (multi_mission.json)

A mission contains:

{
  "primary": "ardrone_1",
  "drones": {
    "ardrone_1": {
      "takeoff_alt": 1.0,
      "waypoints": [
        {"t":0.0, "x":0.0, "y":0.0,   "z":1.0},
        {"t":10.0, "x":1.5, "y":-2.0, "z":1.0}
      ]
    },
    ...
  }
}


Each waypoint contains:

t: timestamp (seconds)

x, y, z: spatial coordinates

📊 6. Output Files Explained
1️⃣ all_trajs.csv

Executed drone trajectory:

time, model, x, y, z

2️⃣ primary_4d.png

Color-coded 4D (XYZ+time) plot.

3️⃣ deconflict_report.txt
Contains:

Number of conflicts

Minimum distance

Average distance

Safety violations

4️⃣ conflicts_planned.json / conflicts_executed.json

Each conflict entry includes:

time, primary_pos, other, other_pos, distance

🎥 7. Demo Video Requirements (Your Submission)

Record a 3–5 minute video showing:

Gazebo simulation

Multi-drone mission execution

CSV trajectory logging

Conflict detection

Visualizations

Final explanation

🧠 8. Scalability (From the PDF Report)

System can scale to tens of thousands of drones using:

Distributed trajectory processing

Message queues (Kafka)

KD-tree batch nearest-neighbor search

GPU-accelerated interpolation

Containerized microservices

Details in FlytBase_assignment.pdf.
