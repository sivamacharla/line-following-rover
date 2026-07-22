"""
Line-Following Rover Simulation

Simulates a differential-drive rover following a curved track using a
5-element IR sensor array (with noise) and a PID controller, plus an
ultrasonic sensor that detects a single obstacle placed on the track.

Used to tune PID gains and validate sensor fusion logic before deploying
to the Arduino firmware (see firmware/rover_firmware.ino).

Outputs:
  - simulation/telemetry.csv   (consumed by the dashboard)
  - simulation/plots.png       (trajectory + error/PID/speed plots)
"""

import csv
import math
import random
from dataclasses import dataclass, field

import matplotlib.pyplot as plt
import numpy as np

# ---------------------------------------------------------------------------
# Track: a wavy path defined as y = f(x). The rover tries to keep its IR
# array centered over this curve.
# ---------------------------------------------------------------------------

def track_y(x: float) -> float:
    return 60 * math.sin(x / 40.0) + 15 * math.sin(x / 11.0)


OBSTACLE_X = 260.0
OBSTACLE_Y = track_y(OBSTACLE_X)
OBSTACLE_RADIUS = 12.0

# Terrain zones along x: simulate patches of poor lighting / uneven surface
# that degrade IR sensor reliability, as described in the project writeup.
TERRAIN_ZONES = [
    (80, 130, 2.5, "low light"),
    (200, 240, 3.5, "uneven surface"),
    (330, 380, 2.0, "low light"),
]


def terrain_noise_multiplier(x: float) -> float:
    for start, end, multiplier, _label in TERRAIN_ZONES:
        if start <= x <= end:
            return multiplier
    return 1.0


# ---------------------------------------------------------------------------
# PID controller
# ---------------------------------------------------------------------------

@dataclass
class PID:
    kp: float
    ki: float
    kd: float
    integral: float = 0.0
    last_error: float = 0.0
    integral_limit: float = 50.0

    def update(self, error: float, dt: float) -> float:
        self.integral += error * dt
        self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))
        derivative = (error - self.last_error) / dt if dt > 0 else 0.0
        self.last_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative


def ema(prev: float, sample: float, alpha: float) -> float:
    return alpha * sample + (1 - alpha) * prev


# ---------------------------------------------------------------------------
# Sensor models
# ---------------------------------------------------------------------------

SENSOR_OFFSETS = [-8, -4, 0, 4, 8]     # mm, left -> right, relative to rover center
SENSOR_WEIGHTS = [-2.0, -1.0, 0.0, 1.0, 2.0]
IR_NOISE_STD = 0.15
SENSOR_WIDTH = 6.0                      # how "wide" a sensor's detection band is


def read_ir_array(rover_x, rover_y, heading, noise_std=IR_NOISE_STD):
    """Return raw (noisy) activation for each of the 5 IR sensors."""
    perp = heading + math.pi / 2
    readings = []
    for offset in SENSOR_OFFSETS:
        sx = rover_x + offset * math.cos(perp)
        sy = rover_y + offset * math.sin(perp)
        track_at_x = track_y(sx)
        dist_from_line = abs(sy - track_at_x)
        activation = max(0.0, 1.0 - dist_from_line / SENSOR_WIDTH)
        activation += random.gauss(0, noise_std)
        readings.append(max(0.0, activation))
    return readings


def read_ultrasonic(rover_x, rover_y, noise_std=0.4):
    dist = math.hypot(rover_x - OBSTACLE_X, rover_y - OBSTACLE_Y) - OBSTACLE_RADIUS
    dist = max(0.0, dist) + random.gauss(0, noise_std)
    return dist if dist < 400 else 400.0


# ---------------------------------------------------------------------------
# Simulation
# ---------------------------------------------------------------------------

@dataclass
class SimConfig:
    kp: float = 34.0
    ki: float = 0.6
    kd: float = 7.0
    base_speed: float = 30.0       # mm per step "speed units"
    dt: float = 0.02
    steps: int = 1200
    obstacle_slow_cm: float = 25.0
    obstacle_stop_cm: float = 8.0
    ir_ema_alpha: float = 0.35
    us_ema_alpha: float = 0.25
    surface_noise_scale: float = 1.0   # >1 simulates rougher terrain / worse lighting


def run_simulation(cfg: SimConfig):
    x, y, heading = 0.0, track_y(0.0), math.atan2(track_y(1) - track_y(0), 1.0)
    pid = PID(cfg.kp, cfg.ki, cfg.kd)

    ir_filtered = [0.0] * 5
    us_filtered = 100.0

    log = []

    for step in range(cfg.steps):
        t = step * cfg.dt

        terrain_mult = terrain_noise_multiplier(x) * cfg.surface_noise_scale
        raw = read_ir_array(x, y, heading, noise_std=IR_NOISE_STD * terrain_mult)
        ir_filtered = [ema(f, r, cfg.ir_ema_alpha) for f, r in zip(ir_filtered, raw)]

        us_raw = read_ultrasonic(x, y)
        us_filtered = ema(us_filtered, us_raw, cfg.us_ema_alpha)

        total = sum(ir_filtered)
        if total < 0.3:
            error = 2.0 if pid.last_error >= 0 else -2.0
        else:
            error = sum(w * a for w, a in zip(SENSOR_WEIGHTS, ir_filtered)) / total

        correction = pid.update(error, cfg.dt)

        if us_filtered <= cfg.obstacle_stop_cm:
            speed_cap = 0.0
        elif us_filtered >= cfg.obstacle_slow_cm:
            speed_cap = cfg.base_speed
        else:
            ratio = (us_filtered - cfg.obstacle_stop_cm) / (cfg.obstacle_slow_cm - cfg.obstacle_stop_cm)
            speed_cap = cfg.base_speed * ratio

        left_speed = max(0.0, speed_cap - correction)
        right_speed = max(0.0, speed_cap + correction)

        forward_speed = (left_speed + right_speed) / 2.0
        turn_rate = (right_speed - left_speed) / 16.0  # 16mm ~ wheel-base proxy

        heading += turn_rate * cfg.dt
        x += forward_speed * math.cos(heading) * cfg.dt
        y += forward_speed * math.sin(heading) * cfg.dt

        log.append({
            "t": t, "x": x, "y": y, "track_y": track_y(x), "heading": heading,
            "error": error, "correction": correction,
            "distance_cm": us_filtered, "speed_cap": speed_cap,
            "left_speed": left_speed, "right_speed": right_speed,
            "terrain_mult": terrain_mult,
        })

    return log


def save_csv(log, path):
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(log[0].keys()))
        writer.writeheader()
        writer.writerows(log)


def plot_results(log, path):
    t = [r["t"] for r in log]
    x = [r["x"] for r in log]
    y = [r["y"] for r in log]
    track = [r["track_y"] for r in log]
    error = [r["error"] for r in log]
    correction = [r["correction"] for r in log]
    distance = [r["distance_cm"] for r in log]

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))

    for start, end, _mult, label in TERRAIN_ZONES:
        axes[0, 0].axvspan(start, end, color="purple", alpha=0.12)
    axes[0, 0].plot(x, track, "--", label="track", color="gray")
    axes[0, 0].plot(x, y, label="rover path")
    axes[0, 0].scatter([OBSTACLE_X], [OBSTACLE_Y], color="red", label="obstacle", zorder=5)
    axes[0, 0].set_title("Trajectory vs. track (shaded = degraded-sensing terrain)")
    axes[0, 0].set_xlabel("x (mm)")
    axes[0, 0].set_ylabel("y (mm)")
    axes[0, 0].legend()

    axes[0, 1].plot(t, error)
    axes[0, 1].set_title("Line error over time")
    axes[0, 1].set_xlabel("t (s)")
    axes[0, 1].set_ylabel("error")

    axes[1, 0].plot(t, correction)
    axes[1, 0].set_title("PID correction (steering output)")
    axes[1, 0].set_xlabel("t (s)")
    axes[1, 0].set_ylabel("correction")

    axes[1, 1].plot(t, distance)
    axes[1, 1].axhline(25, color="orange", linestyle="--", label="slow threshold")
    axes[1, 1].axhline(8, color="red", linestyle="--", label="stop threshold")
    axes[1, 1].set_title("Ultrasonic distance to obstacle")
    axes[1, 1].set_xlabel("t (s)")
    axes[1, 1].set_ylabel("distance (cm)")
    axes[1, 1].legend()

    fig.tight_layout()
    fig.savefig(path, dpi=150)
    print(f"Saved plots to {path}")


def rmse_from_track(log):
    errs = [(r["y"] - r["track_y"]) ** 2 for r in log]
    return math.sqrt(sum(errs) / len(errs))


if __name__ == "__main__":
    cfg = SimConfig()
    log = run_simulation(cfg)
    save_csv(log, "telemetry.csv")
    plot_results(log, "plots.png")
    print(f"Trajectory RMSE vs track: {rmse_from_track(log):.2f} mm")
