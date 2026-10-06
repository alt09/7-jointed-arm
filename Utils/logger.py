import csv
import os
import time
import pandas as pd
from datetime import datetime

import matplotlib.pyplot as plt
import numpy as np
def plot_joint_data(log_file):
    data = pd.read_csv(log_file)

    average_errors=[]
    final_errors=[]
    max_errors=[]

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
        plt.show()

        average_error = joint_data["position_error"].abs().mean()
        average_errors.append(average_error)

        final_data = joint_data.tail(max(1, int(len(joint_data) * 0.1)))  # Last 10% of the data

        final_error = final_data["position_error"].abs().mean()
        final_errors.append(final_error)

        max_error = joint_data["position_error"].abs().max()
        max_errors.append(max_error)

        print(f"Joint {i + 1}:")
        print(f"  Mean absolute error:  {average_error:.4f} rad")
        print(f"  Final average error:  {final_error:.4f} rad")
        print(f"  Maximum error:        {max_error:.4f} rad")
        print()



    average_errors = pd.Series(average_errors, index=range(1,8))
    final_errors = pd.Series(final_errors, index=range(1,8))
    max_errors = pd.Series(max_errors, index=range(1,8))

    plt.figure(figsize=(10, 6))
    plt.bar(average_errors.index, average_errors.values, color='green')
    plt.title('Average Position Error for Each Joint')
    plt.xlabel('Joint')
    plt.ylabel('Average Position Error (rad)')
    plt.grid()
    plt.show()

    plt.figure(figsize=(10, 6))

    plt.bar(final_errors.index, final_errors.values, color='purple')
    plt.title('Final Average Position Error for Each Joint')
    plt.xlabel('Joint')
    plt.ylabel('Final Average Position Error (rad)')
    plt.xticks(range(1, 8))
    plt.grid(axis='y')
    plt.show()

    plt.figure(figsize=(10, 6))
    plt.bar(max_errors.index, max_errors.values, color='red')
    plt.title('Maximum Position Error for Each Joint')
    plt.xlabel('Joint')
    plt.ylabel('Maximum Position Error (rad)')
    plt.xticks(range(1, 8))
    plt.grid(axis='y')
    plt.show()

class RobotLogger:

    def __init__(self, log_dir="logs"):
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
            "actual_velocity"
        ])

        self.start_time = time.perf_counter()

    def log_joint(
        self,
        joint,
        desired_position,
        actual_position,
        actual_velocity
    ):
        elapsed = time.perf_counter() - self.start_time
        error = desired_position - actual_position

        self.writer.writerow([
            elapsed,
            joint,
            desired_position,
            actual_position,
            error,
            actual_velocity
        ])

    def close(self):
        self.file.close()