#!/usr/bin/env python3
"""
7-Jointed Arm Simulation using PyBullet
Structure: Shoulder (3 joints), Elbow (1 joint), Wrist (3 joints)
Similar to Canadarm3
""" 

import pybullet as p
import pybullet_data
import numpy as np
import time

class ArmSimulation:
    def __init__(self, gui=True):
        """Initialize the simulation environment"""
        # Start physics engine
        if gui:
            self.client = p.connect(p.GUI)
        else:
            self.client = p.connect(p.DIRECT)
        
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, 0)
        
        # Load plane for reference
        self.plane_id = p.loadURDF("plane.urdf")
        
        # Load the 7-jointed arm
        arm_urdf = "arm.urdf"
        self.arm_id = p.loadURDF(arm_urdf, basePosition=[0, 0, 1])
        
        # Store joint information
        self.joint_info = {}
        self.joint_indices = []
        self.num_joints = p.getNumJoints(self.arm_id)
        
        # Collect all revolute joints (exclude fixed joints)
        for i in range(self.num_joints):
            joint_info = p.getJointInfo(self.arm_id, i)
            joint_name = joint_info[1].decode('utf-8')
            joint_type = joint_info[2]
            
            if joint_type == p.JOINT_REVOLUTE:
                self.joint_info[joint_name] = {
                    'index': i,
                    'lower_limit': joint_info[8],
                    'upper_limit': joint_info[9],
                    'max_force': joint_info[10],
                    'max_velocity': joint_info[11]
                }
                self.joint_indices.append(i)
        
        print(f"Loaded arm with {len(self.joint_indices)} revolute joints:")
        for joint_name, info in self.joint_info.items():
            print(f"  - {joint_name}: {info['index']}")
        
        # Camera setup for better visualization
        p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=45, cameraPitch=-30, cameraTargetPosition=[0, 0, 0.3])
        
        self.simulation_running = False
    
    def set_joint_angle(self, joint_index, target_angle):
        """Set a joint to a target angle"""
        if isinstance(joint_index, str):
            if joint_index in self.joint_info:
                joint_index = self.joint_info[joint_index]['index']
            else:
                print(f"Joint {joint_index} not found")
                return
        
        p.setJointMotorControl2(
            self.arm_id,
            joint_index,
            p.POSITION_CONTROL,
            targetPosition=target_angle,
            force=100
        )
    
    def set_configuration(self, angles_dict):
        """Set multiple joint angles at once
        
        Args:
            angles_dict: Dictionary mapping joint names/indices to target angles
                        e.g., {'shoulder_joint_1': 0.5, 'elbow_joint': -1.0}
        """
        for joint_name, angle in angles_dict.items():
            self.set_joint_angle(joint_name, angle)
    
    def get_end_effector_position(self):
        """Get the position of the end effector"""
        # End effector is the last link
        end_effector_index = self.num_joints - 1
        state = p.getLinkState(self.arm_id, end_effector_index)
        return state[0]  # Return position (x, y, z)
    
    def run_simulation(self, duration=10.0):
        """Run the simulation for a specified duration"""
        print(f"\nStarting simulation for {duration} seconds...")
        self.simulation_running = True
        
        start_time = time.time()
        
        try:
            while (time.time() - start_time) < duration:
                # Optional: Make the arm move in a simple pattern
                # Uncomment to see the arm in motion:
                # t = time.time() - start_time
                # self.set_joint_angle('shoulder_joint_1', np.sin(t) * 0.5)
                # self.set_joint_angle('elbow_joint', np.cos(t) * 0.5)
                
                p.stepSimulation()
                time.sleep(1.0 / 240.0)  # 240 Hz simulation
                
        except KeyboardInterrupt:
            print("\nSimulation stopped by user")
        
        self.simulation_running = False
    
    def cleanup(self):
        """Disconnect from the physics server"""
        p.disconnect()
        print("Simulation closed")


def main():
    # Create simulation
    sim = ArmSimulation(gui=True)
    
    # Set arm to a neutral configuration
    print("\nSetting neutral configuration...")
    neutral_config = {
        'shoulder_joint_1': 0.0,
        'shoulder_joint_2': 0.0,
        'shoulder_joint_3': 0.0,
        'elbow_joint': 0.0,
        'wrist_joint_1': 0.0,
        'wrist_joint_2': 0.0,
        'wrist_joint_3': 0.0
    }
    sim.set_configuration(neutral_config)
    
    # Run for 5 seconds
    sim.run_simulation(duration=2.0)
    
    # Move to a different configuration
    print("\nMoving to new configuration...")
    new_config = { #TODO: make this adjustable via commands 
        'shoulder_joint_1': 0.0,
        'shoulder_joint_2': 0.0,
        'shoulder_joint_3': 0.0,
        'elbow_joint': 0.0,
        'wrist_joint_1': 0.0,
        'wrist_joint_2': 0.0,
        'wrist_joint_3': 0.0
    }
    sim.set_configuration(new_config)
    
    sim.run_simulation(duration=5.0)
    
    # Get end effector position
    end_pos = sim.get_end_effector_position()
    print(f"\nEnd effector position: {end_pos}")
    
    # Keep GUI window open
    print("\nSimulation window will stay open. Press Ctrl+C to close.")
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    
    sim.cleanup()


if __name__ == "__main__":
    main()
