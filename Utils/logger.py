import csv
import os
import time
from datetime import datetime


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
            "actual_velocity",
            "control_torque"
        ])

        self.start_time = time.perf_counter()

    def log_joint(
        self,
        joint,
        desired_position,
        actual_position,
        actual_velocity,
        control_torque
    ):
        elapsed = time.perf_counter() - self.start_time
        error = desired_position - actual_position

        self.writer.writerow([
            elapsed,
            joint,
            desired_position,
            actual_position,
            error,
            actual_velocity,
            control_torque
        ])

    def close(self):
        self.file.close()