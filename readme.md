# Canadarm3 Simulation

A personal robotics simulation project inspired by **Canadarm3**, the robotic arm being developed for the Lunar Gateway.

The goal of this project is to explore robotic arm modeling, kinematics, computer vision, and autonomous positioning using a simulated environment.

## How to Run

1. Clone the repository.
2. Install the required Python dependencies:

```bash
pip install -r requirements.txt
```
3. Run the simulation

```bash
python Main.py
```
## Features
- 3D robotic arm simulation using pybullet
- URDF robot model
- Stereo computer vision 
- 3D triangulation
- Triangulation error checking to reject unreliable measurements
- Automatic aiming toward a detected target
- OpenCV target detection
- Inverse kinematics
- Forward kinematics
- Position error checking
- Last-known-target tracking when the target is temporarily out of sight


## Reference Data:
- source: https://www.asc-csa.gc.ca/eng/iss/canadarm2/canadarm-canadarm2-canadarm3-comparative-table.asp 
- Length = 8.5m long arm 
- Mass = 1076kg
- Diameter = 23cm
- Composition = carbon fiber composite
- joint rotation = 358°(estimation)

## Estimated Simulation Configuration:
```
GATEWAY / BASE
    │
    │
    ▼
┌───────────────┐
│ SHOULDER      │
│ J1 J2 J3      │
└──────┬────────┘
       │
       │ 520 mm shoulder assembly
       │
       ▼
════════════════════════════════════
       BOOM A
       3,350 mm
════════════════════════════════════
       │
       ▼
   ┌─────────┐
   │ ELBOW   │
   │   J4    │
   └────┬────┘
        │
        │
════════════════════════════════════
       BOOM B
       3,450 mm
════════════════════════════════════
        │
        ▼
   ┌───────────────┐
   │ WRIST         │
   │ J5 J6 J7      │
   └──────┬────────┘
          │
          │ 300 mm
          ▼
     ┌─────────┐
     │   LEE   │
     │  HAND   │
     └─────────┘
```

## Note:
The dimensions and joint arrangements shown above are my estimates created only for this project. This project is intended as an educational simulation than a replica of the flight system 

## Technologies
- Python
- PyBullet
- NumPy
- OpenCV
- URDF
- Git 

## What I Learned
- Coordinate frames and transformations
- Stereo vision and triangulation
- Computer vision with OpenCV
- URDF robot modeling
- Quaternions vs Euler angles
- Inverse kinematics
- Forward kinematics
- Numpy
- 3D triangulation

# accuracy  
## Inverse kinematics
- Number of tests: 10,000
- Reachable targets: 9,108/9,816
- Reachable target success rate:  91.8%
- Unreachable targets: 892/10,000 (8.9%)
  
```The unreachable targets are most likely caused by physically impossible positions, such as targets located inside the robotic arm or outside its reachable workspace.```

### Pose
- Average Position Error: 0.000051 m
- Max Position Error: 0.000099 m
### Orientation
- Average Orientation Error: 0.000067 rad (0.0038°)
- Max Orientation Error: 0.000141 rad (0.0081°)
  
## Vision
### Set up

- Number of tests: 180
- Object: 0.1m radius sphere
- best before 10.4037 m

```The error can depend on the shape of the object and where the center is located. A circle and sphere are the best shapes to test the accuracy)```

### Filtered 
- Mean error: 0.08363053329379369 m
- Median error: 0.07358221298563601 m
- Maximum error: 0.15566874517459925 m 
- Acceptance rate: 98.33333333333333%
- Number of acceptable tests: 177/180
### Raw
- Mean error: 0.08383519893634735 m
- Median error: 0.07358224200916538 m
- Maximum error: 0.15566874517459925 m
- Number of acceptable tests: 180

the filer and raw results are very similar, this proves that the triangular error filter rejects only a small number of measurements. 
