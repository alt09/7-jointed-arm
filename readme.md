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
- Stereo computer vision for estimating a target 3D position
- Automatic aiming toward a detected target
- OpenCV target detection

## DATA:
source: https://www.asc-csa.gc.ca/eng/iss/canadarm2/canadarm-canadarm2-canadarm3-comparative-table.asp 
Length = 8.5m long arm 
Mass = 1076kg
Diameter = 23cm
Composition = carbon fibre composite
joint rotation = 358°(estimation)

## how I think it is(estimation):
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

## NOTE:
The dimensions and joint arrangements shown above are my estimates created only for this project. This project is intended as an educational simulation than a replica of the flight system 

##Technologies
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
- quaternions vs Euler angles
- always check radiants or degrees :C

## thanks 


- thanks for my robotics club that teached me english and coding
- my mom
- my dad
- my brother
- my cousins
- my friend Ben
- my other friends
- the family of my frieds
- my goat monkey D luffy
- gold D roger
- joy boy
- special thanks to eichiro oda
