import csv
import os
import time
import pandas as pd
from datetime import datetime

import matplotlib.pyplot as plt
import numpy as np

def plot_joint_data(log_file):
    """
    Plots the joint data from the log file.
    and generates plots for position, error, and joint data.
    Args:
        log_file (str): Path to the log file.
    """
    # Read the log file into a pandas DataFrame
    data = pd.read_csv(log_file)

    # Initialize lists to store average, final, and maximum errors for each joint
    average_errors=[]
    final_errors=[]
    max_errors=[]

    # Extract the 3D position and target position data from the DataFrame
    position_3d_data = data['additional_info']
    target_position_data = data['target_position']

    # Convert the stored 3D positions into separate X, Y, Z values
    position_3d = position_3d_data.apply(eval)
    target_position_3d = target_position_data.apply(eval)

    # Extract X, Y, Z positions for both actual and target positions
    x_position = position_3d.apply(lambda p: p[0])
    y_position = position_3d.apply(lambda p: p[1])
    z_position = position_3d.apply(lambda p: p[2])
    target_x_position = target_position_3d.apply(lambda p: p[0])
    target_y_position = target_position_3d.apply(lambda p: p[1])
    target_z_position = target_position_3d.apply(lambda p: p[2])

    
    # Create a 3D position plot and a 3D position error plot
    plt.figure(figsize=(10, 6))
    plt.plot(data['time'], x_position, label='X Position')
    plt.plot(data['time'], y_position, label='Y Position')
    plt.plot(data['time'], z_position, label='Z Position')
    plt.plot(data['time'], target_x_position, label='Target X Position', linestyle='--', color='blue')
    plt.plot(data['time'], target_y_position, label='Target Y Position', linestyle='--', color='orange')
    plt.plot(data['time'], target_z_position, label='Target Z Position', linestyle='--', color='green')

    plt.title('3D Position vs Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    plt.legend()
    plt.grid()
    plt.savefig("Media/3D_Position_vs_Time.png")  # Save the figure as a PNG file
    plt.show()

    # Create a 3D position error plot
    x_error = x_position - target_x_position
    y_error = y_position - target_y_position
    z_error = z_position - target_z_position

    plt.figure(figsize=(10, 6))
    plt.plot(data['time'], x_error, label='X Error')
    plt.plot(data['time'], y_error, label='Y Error')
    plt.plot(data['time'], z_error, label='Z Error')

    plt.title('3D Position Error vs Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Error (m)')
    plt.legend()
    plt.grid()
    plt.savefig("Media/3D_Position_Error_vs_Time.png")  # Save the figure as a PNG file
    plt.show()

    # Generate plots for each joint's desired vs actual position and calculate average, final, and maximum errors
    for i in range(7):

        joint_data = data[data['joint'] == i]

        plt.figure(figsize=(10, 6))
        plt.plot(joint_data['time'], joint_data['desired_position'], label='Desired Position', color='blue')
        plt.plot(joint_data['time'], joint_data['actual_position'], label='Actual Position', color='orange')
        plt.title(f'Joint {i} Position Over Time')
        plt.xlabel('Time (s)')
        plt.ylabel('Position (rad)')
        plt.legend()
        plt.grid()
        plt.savefig(f"Media/Joint_{i}_Position_Over_Time.png")  # Save the figure as a PNG file
        plt.show()

        average_error = joint_data["position_error"].abs().mean()
        average_errors.append(average_error)

        final_data = joint_data.tail(max(1, int(len(joint_data) * 0.1)))  # Last 10% of the data

        final_error = final_data["position_error"].abs().mean()
        final_errors.append(final_error)

        max_error = final_data["position_error"].abs().max()
        max_errors.append(max_error)

    # Convert the error lists to pandas Series for easier plotting
    average_errors = pd.Series(average_errors, index=range(1,8))
    final_errors = pd.Series(final_errors, index=range(1,8))
    max_errors = pd.Series(max_errors, index=range(1,8))

    # Generate bar plots for average, final, and maximum errors for each joint
    plt.figure(figsize=(10, 6))
    plt.bar(average_errors.index, average_errors.values, color='green')
    plt.title('Average Position Error for Each Joint')
    plt.xlabel('Joint')
    plt.ylabel('Average Position Error (rad)')
    plt.grid(axis='y')
    plt.savefig("Media/Average_Position_Error_for_Each_Joint.png")  # Save the figure as a PNG file
    plt.grid()
    plt.show()

    # Generate bar plots for final and maximum errors for each joint
    plt.figure(figsize=(10, 6))
    plt.bar(final_errors.index, final_errors.values, color='purple')
    plt.title('Final Average Position Error for Each Joint')
    plt.xlabel('Joint')
    plt.ylabel('Final Average Position Error (rad)')
    plt.xticks(range(1, 8))
    plt.grid(axis='y')
    plt.savefig("Media/Final_Average_Position_Error_for_Each_Joint.png")  # Save the figure as a PNG file
    plt.show()

    # Generate bar plots for maximum errors for each joint
    plt.figure(figsize=(10, 6))
    plt.bar(max_errors.index, max_errors.values, color='red')
    plt.title('Maximum Position Error for Each Joint')
    plt.xlabel('Joint')
    plt.ylabel('Maximum Position Error (rad)')
    plt.xticks(range(1, 8))
    plt.grid(axis='y')
    plt.savefig("Media/Maximum_Position_Error_for_Each_Joint.png")  # Save the figure as a PNG file
    plt.show()


class RobotLogger:
    """
    A class for logging robot joint data during a simulation.
    """

    def __init__(self, log_dir="logs"):
        """
        Initializes the RobotLogger.
        Args:
            log_dir (str): Directory where the log files will be saved.
        """
        os.makedirs(log_dir, exist_ok=True)

        timestamp = datetime.now().strftime(
            "%Y-%m-%d_%H-%M-%S"
        )

        self.file = open(
            os.path.join(
                log_dir, f"canadarm3_{timestamp}.csv"
            ),
            "w",
            newline=""
        )

        self.writer = csv.writer(self.file)

        self.writer.writerow([
            "time",
            "joint",
            "desired_position",
            "actual_position",
            "position_error",
            "actual_velocity",
            "additional_info",
            "target_position"
        ])

        self.start_time = time.perf_counter()

    def log_joint(
        self,
        joint,
        desired_position,
        actual_position,
        actual_velocity,
        additional_info=None,
        target_position=None,
        end_time=None
    ):
        """
        Logs the joint data to the CSV file.
        Args:
            joint (int): Joint index.
            desired_position (float): Desired joint position.
            actual_position (float): Actual joint position.
            actual_velocity (float): Actual joint velocity.
            additional_info (any, optional): Additional information to log.
            target_position (list, optional): Target position of the end effector.
            end_time (float, optional): If specified, logging will stop after this time (in seconds).
        """
        elapsed = time.perf_counter() - self.start_time
        error = desired_position - actual_position

        # If an end_time is specified and the elapsed time exceeds it, stop logging and close the file
        if end_time is not None and elapsed > end_time:

            print(f"Logging stopped after {end_time} seconds.")
            self.close()

        # Log the joint data to the CSV file
        self.writer.writerow([
            elapsed,
            joint,
            desired_position,
            actual_position,
            error,
            actual_velocity,
            additional_info,
            target_position
        ])

    def close(self):
        """
        Closes the CSV file.
        """
        self.file.close()