# Verification Report

## 7-Jointed Robotic Arm Simulation

**Document ID:** VR-001  
**Version:** 1.0 
**Status:** Approved   
**Project:** 7 Jointed Arm  
**System Type:** Robotic arm simulation and software verification

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

## 3. Verification Environment

### 3.1 Software Environment

| Library | Version | 
|---|---|
| pybullet | 3.2.7 | 
| numpy | 2.5.2 | 
| pandas | 3.0.6 | 
| opencv-python | 5.0.0.93 | 
| matplotlib | 3.11.1 | 

### 3.2 Test Configuration 

| Configuration | Version |
|---|---|
| Python | 3.14.0 |
| Git commit number and branch | Feat#4-systems-engineering : commit #108 | 
| OS | Windows 11 Home | 25H2 |

## 4. Verification Summary

| Metric | Result |
|---|---|
| Total planned test cases | [10201] |
| Tests executed | [10201] |
| Tests passed | [10138] |
| Tests failed | [0] |
| Tests blocked | [63] |
| Requirements verified | [21] |
| Requirements not verified | [0] |
| Open defects | [4] |

## 5. Test Results

### 5.1 Kinematics

| ID | Name | Acceptance criteria | Actual result | Status
|---|---|---|---|---|
| NEED-001 | Main objective | The arm is within 3cm of the target position | Less than 0.0001m | Pass
| KIN-001 | Forward kinematics | FK error should be less than the error limit | Less than 0.0001m | Pass
| KIN-002 | Inverse kinematics | IK error should be less than the error limit, and should respect joint limits and impossible solutions | Less than 0.0001m | Pass
| KIN-003 | Unreachable target | The IK should report failure when it can not find a valid solution | It returned None when an unreachable target was given | Pass
| Perf-001 | FK accuracy | FK solutions should mantein a maximum error below 1cm | Less than 0.0001m | Pass
| Perf-002 | FK accuracy |IK solutions should mantein a maximum error below 1cm | Less than 0.0001m | Pass

### 5.2 Vision 
| ID | Name | Acceptance criteria | Actual result | Status
|---|---|---|---|---|
| NEED-002 | Vision need | Target position should be within 10cm error | Max error of 9.88cm | Pass
| VIS-001 | Stereo target position | The vision subsystem should estimate the 3D position of the target using the simulated cameras, satisfying the triangulation error and an error tolerance | Satisfied the triangulation error, but a max position error of 9.88cm | Pass
| VIS-002 | Measurement validation | The vision subsystem should reject measurements when a triangulation error exceeds 0.05 | The system rejected measurements with high triangulation error| Pass
| PERF-003 | Vision accuracy | the maximum target 3D pose error should be below 10 cm | max position error of 9.88cm | Pass

### 5.3 Trajectory 

| ID | Name | Acceptance criteria | Actual result | Status
|---|---|---|---|---|
| NEED-003 | Trajectory need | The arm should follow trajectories within defined motion limits | It follows a trajectory without passing the motion limits | Pass 
| MOT-001 | Joint velocity limit | The calculated joint velocities should not exceed the joint velocity limits 1 rad/second | The trajectory is generated following the motion limits | Pass
| MOT-002 | Joint acceleration limit | The calculated joint acceleration should not exceed the joint acceleration limits of 1 rad/second^2 | The trajectory is generated following the motion limits | Pass

### 5.4 General 

| ID | Name | Acceptance criteria | Actual result | Status
|---|---|---|---|---|
| NEED-004 | Invalid measurements | The system should handle invalid measurements and failed solutions. | I never crashed or gave wrong data | Pass
| NEED-005 | Logging need | The logger should show graphs showing the performance of the arm | The logger graphs and data are clearly shown and can be zoom in and out | Pass
| SYS-001 | Robot Configuration | The robot model in the simulation should contain seven independent joints. | The robot model appears in the simulation and has 7 movable joints | Pass
| SYS-002 | Target to motion pipeline | The robot arm should reach the target position | The robot arm reaches the target within 1cm | Pass
| VIS-003 | Target loss handling | The vision should be able to retain the last valid target 3D pose when the target is undetectable | The system recognizes when the target is lost and stores the last position | Pass 
| CTRL-001 | End effector settling | The controller should bring the end effector to the target position within the settling time. | The controller settles within the settling time | Pass
| LOG-001 | Trajectory logging | The log file contains the 3D target position and the actual 3D position as well as the position and target position of each joint | The log file shows the data necessary to analyze the arm trajectory performance| Pass
| PERF-004 | Settling time | the controller should settle within 0.2 seconds | The controller settles within the settling time | Pass

## 6. Defects

| Defect ID | Test ID | Description | Severity | Resolution | Retest status |
|---|---|---|---|---|---|
| DEF-001 | Need-002 | The average vision error is below 10 cm, but it reaches at a max error of 15.5 cm or higher | medium | Vision tuning and complementation with other way to detect the target | Closed 
| DEF-002 | VIS-001 | The triangulation error is satisfied, but the position error is not | medium | Vision tuning and complementation with other way to detect the target | Closed
| DEF-003 | PERF-003 |  max error of 15.5 cm or higher | medium | Vision tuning and complementation with other way to detect the target | Closed 

## 7. Conclusions 

The robotic arm can reach the target position with an accuracy of less than 1cm, respecting the motion limits and the joint limits, at the same time it generates graphs and logs data for future analysis, but the vision subsystem is not as accurate and consistent as the rest of the subsystems, generating high error measurements, feeding inaccurate information to the rest of the subsystems, but only happens on extreme cases like long distance detection or partial view of the target. 

## 8. Reproducibility 

- Test source files and execution commands.
- Raw CSV logs and numerical summaries.
- Trajectory and error plots.
- Relevant simulation configuration.
- Git commit SHA used for testing.