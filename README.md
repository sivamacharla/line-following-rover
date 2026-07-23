# Autonomous Line-Following Rover

An autonomous rover that follows a line across varied terrain using an IR sensor
array and an ultrasonic sensor, controlled by a PID loop with EMA-filtered
sensor fusion. This repo contains the Arduino firmware, a Python simulation
used to tune the control constants before deploying to hardware, and a
telemetry dashboard for visualizing rover behavior (from simulation logs or
live over serial).

## Structure

```
line-following-rover/
├── firmware/
│   └── rover_firmware.ino   # Arduino C++ firmware: PID + sensor fusion + motor control
├── firmware-stm32/
│   ├── README.md            # CubeMX pin/peripheral config + build steps
│   └── Core/                # HAL port for NUCLEO-F401RE (drop into a CubeIDE project)
├── simulation/
│   ├── rover_sim.py         # Physics + sensor-noise simulation, generates telemetry.csv + plots.png
│   ├── tune_pid.py          # Grid-search PID tuning against the simulation
│   └── requirements.txt
└── dashboard/
    └── index.html           # Self-contained telemetry dashboard (CSV load or live Web Serial)
```

## How it works

**Sensing.** Five IR reflectance sensors read the line position; an HC-SR04
ultrasonic sensor detects obstacles/terrain edges ahead. Both are smoothed
with an exponential moving average (EMA) filter to reject noise and avoid
PID overcorrection.

**Control.** The IR array produces a weighted line-position error (`-2..+2`).
A PID loop turns that error into a steering correction that's applied
differentially to the left/right motors. The ultrasonic reading independently
caps the base speed as obstacles get closer (sensor fusion: two sensor
modalities combine to produce one motor command).

**Tuning.** `simulation/rover_sim.py` models rover kinematics, a curved
track, sensor noise, and an obstacle, then plots trajectory/error/PID output.
`tune_pid.py` grid-searches Kp/Ki/Kd and scores each set by trajectory RMSE,
so gains can be tuned in software before touching real hardware — mirroring
the MATLAB-based workflow used in the original project.

## Running the simulation

```bash
cd simulation
pip install -r requirements.txt
python rover_sim.py        # writes telemetry.csv + plots.png
python tune_pid.py         # grid-search for best Kp/Ki/Kd
```

Copy the winning gains into `KP`/`KI`/`KD` in both `rover_firmware.ino` and
`SimConfig` in `rover_sim.py`.

## Using the dashboard

Open `dashboard/index.html` in a browser (Chrome/Edge recommended):

- **Load CSV** — load `simulation/telemetry.csv` to review a simulation run.
- **Connect Arduino (Live)** — uses the Web Serial API to read live telemetry
  directly from the rover over USB while it runs. The firmware streams CSV
  lines (`time,error,correction,distance,speedCap`) at 115200 baud; the
  dashboard parses and charts them in real time.

## Firmware wiring (reference)

| Signal        | Pin |
|---------------|-----|
| IR sensors    | A0–A4 |
| Ultrasonic TRIG | D8 |
| Ultrasonic ECHO | D9 |
| Left motor PWM (ENA) | D5 |
| Left motor IN1/IN2   | D4 / D3 |
| Right motor PWM (ENB)| D6 |
| Right motor IN3/IN4  | D7 / D2 |

Adjust pins in `rover_firmware.ino` to match your driver board.

## STM32 Nucleo-F401RE port

`firmware-stm32/` has an HAL-based port of the same control logic for the
NUCLEO-F401RE — see [`firmware-stm32/README.md`](firmware-stm32/README.md)
for the CubeMX pin/peripheral configuration and build steps. It hasn't been
build-verified (no ARM toolchain in the environment it was written in), so
expect to fix compiler errors on first build against your generated project.
