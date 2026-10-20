# System Requirements Specification

## 7-Jointed Robotic Arm Simulation

**Document ID:** SRS-001  
**Version:** 1.1 
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

## 3. System Overview

The system receives simulated camera feedback, detects a target, estimates its 3D position, generates a joint configuration, generates a trajectory, moves the robot arm to reach the target.

Simultaneously, it records measurements and errors to analyze them later.

## 4. Assumptions and Constraints

- The simulated robotic arm has seven independently represented joints.
- PyBullet provides the simulation environment and physics engine.
- The project's FK and IK implementations must not depend on PyBullet's built-in FK or IK calculations.
- Joint positions, velocities, and accelerations must use documented units.
- Target positions and end effector poses must use explicitly defined coordinate frames.
- Performance requirements must be evaluated using documented test datasets and reproducible procedures.
- Only 1 target at a time should be presented to the robotic arm

## 5. Stakeholder Needs

| ID | Stakeholder need | Acceptance criteria | Verification method
|---|---|---|---|
| NEED-001 | The arm should reach valid target poses. | The arm is within 3cm of the target position | Test
| NEED-002 | The system should estimate target positions using stereo vision. | Target position should be within 10cm error | Test  
| NEED-003 | The arm should follow trajectories within defined motion limits. | The arm follows each step of the trajectory | Test
| NEED-004 | The system should handle invalid measurements and failed solutions. | The system should not crash after receiving invalid measurements | Demonstration
| NEED-005 | The developer should be able to reproduce and evaluate system performance. | The logger should show graphs showing the performance of the arm  | Inspection/Demonstration

## 6. System Requirements

### 6.1 System functionality

| ID | Name | Acceptance criteria | Verification method
|---|---|---|---|
| SYS-001 | Robot Configuration | The robot model in the simulation should contain seven independent joints. | Inspection
| SYS-002 | Target to motion pipeline | The robot arm should reach the target position | Demonstration


### 6.2 Kinematics

| ID | Name | Acceptance criteria | Verification method
|---|---|---|---|
| KIN-001 | Forward kinematics | FK error should be less than the error limit | Analysis/Test
| KIN-002 | Inverse kinematics | IK error should be less than the error limit, and should respect joint limits and impossible solutions | Test
| KIN-003 | Unreachable target | The IK should report failure when it can not find a valid solution | Test

### 6.3 Computer vision

| ID | Name | Acceptance criteria | Verification method
|---|---|---|---|
| VIS-001 | Stereo target position | The vision subsystem should estimate the 3D position of the target using the simulated cameras, satisfying the triangulation error and an error tolerance | Test
| VIS-002 | Measurement validation | The vision subsystem should reject measurements when a triangulation error exceeds 0.05| Test
| VIS-003 | Target loss handling | The vision should be able to retain the last valid target 3D pose when the target is undetectable | Inspection

### 6.4 Trajectory generation and control

| ID | Name | Acceptance criteria | Verification method
|---|---|---|---|
| MOT-001 | Joint velocity limit | The calculated joint velocities should not exceed the joint velocity limits | Analysis
| MOT-002 | Joint acceleration limit | The calculated joint acceleration should not exceed the joint acceleration limits | Analysis
| CTRL-001| End effector settling | The controller should bring the end effector to the target position within the settling time. | Test

### 6.5 Logging and reliability

| ID | Name | Acceptance criteria | Verification method
|---|---|---|---|
| LOG-001 | Trajectory logging | The log file contains the 3D target position and the actual 3D position as well as the position and target position of each joint | Analysis

## 7. Performance Requirements

| ID | Name | Acceptance criteria | Verification method
|---|---|---|---|
| PERF-001 | FK accuracy | FK solutions should mantein a maximum error below 1cm | Analysis
| PERF-002 | IK accuracy | IK solutions should mantein a maximum error below 1cm | Analysis
| PERF-003 | Vision accuracy | the maximum target 3D pose error should be below 10 cm | Test
| PERF-004 | Settling time | the controller should settle within 0.2 seconds | Analysis

## 8. Verification and Acceptance
undetermined,
Each system requirement is linked to one or more verification procedures and annotated in the requirements traceability.cvs file.

A requirement shall be marked **Pass** only after its defined acceptance criteria have been evaluated

Failed or incomplete tests shall be documented with their limitations and current status in the notes and marked as **Failed**.

**Not Run** Verification has not been executed

**Blocked**	Verification cannot proceed because of an unresolved dependency or setup issue

## 9. Document Change History

| Version | Description |
|---|---|
| 1.0 | First of the system requirements specification. |
| 1.1 | First review changes of the system requirements. |

