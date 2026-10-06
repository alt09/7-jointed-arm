import csv
import os
import time
import pandas as pd
from datetime import datetime

import matplotlib.pyplot as plt
import numpy as np
def plot_joint_data(log_file):
    data = pd.read_csv(log_file)

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

    average_errors = data.groupby('joint')['position_error'].apply(
        lambda x: x.abs().mean()
    )   
    plt.figure(figsize=(10, 6))
    plt.bar(average_errors.index, average_errors.values, color='green')
    plt.title('Average Position Error for Each Joint')
    plt.xlabel('Joint')
    plt.ylabel('Average Position Error (rad)')
    plt.grid()
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