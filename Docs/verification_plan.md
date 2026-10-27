# Verification Plan

## 7-Jointed Robotic Arm Simulation

**Document ID:** VPLAN-001  
**Version:** 1.0
**Status:** Approved  
**Project:** 7-Jointed Arm  
**Related Document:** SRS-001 — System Requirements Specification

## 1. Purpose
 
The purpose of this project is to develop a 7 jointed robotic arm simulation that can demonstrate kinematics, computer stereo vision, trajectory generation and motion control. 

## 2. Scope

### 2.1 Included in scope

- 7 jointed arm model
- Simulated environment using PyBullet.
- Stereo vision and 3D target triangulation.
- Forward kinematics (FK).
- Inverse kinematics (IK).
- Joint trajectory generation.
- Joint level motion control.
- Target tracking.
- Data logging

### 2.2 Excluded from scope

- Physical robot hardware.
- Flight qualified software development.
- Spacecraft certification.
- Validation against actual Canadarm3 flight hardware or operational specifications.

## 3. Verification methods

| Method | Description | 
|---|---|
| Test | Execute software with controlled variables and compared results against the acceptance criteria |
| Analysis | Mathematically evaluate a property |
| Inspection | Examine source code |
| Demonstration | Visually observe a behavior | 

## 4. Verification Environment

### 4.1 Software Environment

| Library | Version | 
|---|---|
| pybullet | 3.2.7 | 
| numpy | 2.5.2 | 
| pandas | 3.0.6 | 
| opencv-python | 5.0.0.93 | 
| matplotlib | 3.11.1 | 

### 4.2 Test Configuration 

| Configuration | Version |
|---|---|
| Python | 3.14.0 |
| Git commit hash and branch | a4f21efb7623a6645f80af2ccc28ee695c9ea4e4 : Feat#4-systems-engineering | 
| OS | Windows 11 Home | 25H2 |

## 5. General Verification procedure 

1. Install the required dependencies.
2. Execute the automated tests.
3. Record the test inputs, outputs, errors and configuration.
4. Compare the results against the acceptance criteria.
5. Mark the requirements as **Pass**, **Fail**, **Not run** or **Blocked**.
6. Record failures and evidence in `Docs/verification_report.md`.
7. Update `Docs/requirements_traceability.csv`

## 6. Test Procedures

### 6.1 Robot kinematics
**Test ID:** Test-kinematics-001   
**Requirements linked:** NEED-001, KIN-001, KIN-002, KIN-003, PERF-001, PERF-002  
**Method:** Test and Analysis    

**Objective:** Verify implementations related to the robot kinematics

**Procedure:**
1. Generates 10000 random target positions.
2. Calculate a set of joint configurations using IK for each target position.
3. Calculates the end effector position using FK.
4. Calculate the position and orientation error. 
5. Record the test number, pose error, orientation error, calculated position and target position and orientation.

**Measurements:**
- Total test cases.
- Number of reachable targets.
- Number of successful solutions.
- Reachable-target success rate.
- Mean and maximum position error.
- Mean and maximum orientation error.

**Acceptance criteria:**
- Solutions are within the error tolerance.
- Returned joint angles respect joint limits.
- Unreachable targets are reported as failed attempts.

**Evidence:** Automated test logs.

### 6.2 Vision 

**Test ID:** Test-vision-001   
**Requirements linked:** NEED-002, VIS-001, VIS-002, PERF-003
**Method:** Test and analysis

**Objective:** Verify implementations related to the simulated vision system 

**Procedure:**
1. Generates 100 random targets in different positions.
2. Generates a new camera system.
3. Calculates the distance between the camera and the target.
4. Calculate the position error. 
5. Record the test number, pose error, distance between the target and the camera, calculated target position, actual target position and if the target can be seen or not.

**Measurements:**
- Total test cases.
- Number of valid detections.
- Number of successful solutions.
- Target distance and test configuration.
- Mean and maximum 3D pose error.

**Acceptance criteria:**
- Solutions are within the error tolerance.
- Targets out of sight are reported.

**Evidence:** Automated test logs.


### 6.3 Trajectory generation 

**Test ID:** Test-trajectory-001   
**Requirements linked:** NEED-003, MOT-001, MOT-002       
**Method:** Test and Analysis    

**Objective:** Verify implementations related to the trajectory generation, including PID controller

**Procedure:**
1. Generates 100 sets of 7 random target joint configurations.
2. Generate a trajectory for each set.
3. Record the test number, time it takes to settle in position, position error, calculated joint position, target positions, joint velocity and acceleration.

**Measurements:**
- Total test cases.
- Number of successful solutions.
- Mean and maximum settling time.
- Mean and maximum position error.
- Mean and maximum joint velocity.
- Mean and maximum joint acceleration.

**Acceptance criteria:**
- Solutions are within the error tolerance.
- Returned joint angles respect joint limits.
- Solutions slower than the settling time tolerance are reported as failed attempts.

**Evidence:** Automated test logs.

### 6.4 General Test 

**Test ID:** Test-general-001   
**Requirements linked:** NEED-003, MOT-001, MOT-002, CTRL-001, PERF-004   
**Method:** Analysis, inspection and Demonstration    

**Objective:** Verify the full system

**Procedure:**
1. Run sim.py.
2. Generate a trajectory to the center of the target.
3. Review the log data to find any anomaly in the robot's behavior.

**Measurements:**
- Time.
- Joint.
- Desired position.
- Actual position.
- Position error.
- Actual velocity.
- End effector position.
- Target position.

**Acceptance criteria:**
- No anomalies appear on the logs or the simulated behavior of the robot 
- Smooth graphs appear on the logs
- The Robot reaches the target position within 3cm 

**Evidence:** Automated logs.

## 7. Test result classification 
| Status | Definition |
|---|---|
| Pass | All acceptance criteria is satisfied |
| Fail | One or more acceptance criteria is not satisfied | 
| Not run | Verification has not been executed |
| Blocked | Verification cannot proceed |

## Revision History 

| Version | Date | Description |
|---|---|---|
| 1.0  | 2026-10-20 | Initial verification plan draft |

