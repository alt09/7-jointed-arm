import numpy as np

def generate_trajectory(q_start, q_goal, max_velocity, max_acceleration, dt=0.02):
    q_start = np.array(q_start, dtype = np.float64)
    q_goal = np.array(q_goal, dtype = np.float64)

    vmax = np.array(max_velocity, dtype = np.float64)
    amax = np.array(max_acceleration, dtype = np.float64)

    if dt <= 0 or np.any(vmax <= 0) or np.any(amax <= 0):

        raise ValueError("dt, velocity, and acceleration must be positive")

    delta = q_goal - q_start
    distance = np.abs(delta)

    if np.all(distance < 1e-10):

        return [{
            "time": 0.0,
            "position": q_goal.copy(),
            "velocities": np.zeros_like(q_goal),
            "accelerations": np.zeros_like(q_goal)
        }]

    moving = distance > 1e-10

    V = np.min(vmax[moving] / distance[moving])
    A = np.min(amax[moving] / distance[moving])

    if V**2 / A < 1.0:

        peak_velocity = np.sqrt(A)
        t_accel = peak_velocity / A
        t_cruise = 0.0
    else:
        
        peak_velocity = V
        t_accel = V / A
        t_cruise = (1.0 - V**2 / A) / V
    duration = 2 * t_accel + t_cruise

    times = np.arange(0.0, duration, dt)
    times = np.append(times, duration)

    trajectory = []

    for t in times:
        if t < t_accel:

            s = 0.5 * A * t**2
            sd = A * t
            sdd = A
        elif t < t_accel + t_cruise:
            s = 0.5 * A * t_accel**2 + peak_velocity * (t - t_accel)
            sd = peak_velocity
            sdd = 0.0
        elif t < duration:

            remaining = duration - t
            s = 1.0 - 0.5 * A * remaining**2
            sd = A * remaining
            sdd = -A
        else:
            s = 1.0
            sd = 0.0
            sdd = 0.0

        trajectory.append({
            "time": float(t),
            "positions": q_start + s * delta,
            "velocities": sd * delta,
            "accelerations": sdd * delta
        })
    return trajectory

