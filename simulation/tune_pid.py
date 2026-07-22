"""
Grid-search PID tuning against the rover simulation.

Sweeps Kp/Ki/Kd combinations, scores each run by trajectory RMSE against
the track (lower is better), and reports the best-performing gains —
mirroring the iterative, data-driven tuning process described in the
project writeup.
"""

import itertools

from rover_sim import SimConfig, run_simulation, rmse_from_track

KP_RANGE = [12.0, 18.0, 22.0, 28.0, 34.0]
KI_RANGE = [0.0, 0.3, 0.6, 1.0]
KD_RANGE = [4.0, 7.0, 9.5, 13.0]


def main():
    results = []
    combos = list(itertools.product(KP_RANGE, KI_RANGE, KD_RANGE))
    print(f"Evaluating {len(combos)} PID combinations...")

    for kp, ki, kd in combos:
        cfg = SimConfig(kp=kp, ki=ki, kd=kd)
        log = run_simulation(cfg)
        score = rmse_from_track(log)
        results.append((score, kp, ki, kd))

    results.sort(key=lambda r: r[0])

    print("\nTop 10 gain sets (lowest RMSE = best tracking):")
    print(f"{'RMSE (mm)':>10} {'Kp':>8} {'Ki':>8} {'Kd':>8}")
    for score, kp, ki, kd in results[:10]:
        print(f"{score:10.2f} {kp:8.2f} {ki:8.2f} {kd:8.2f}")

    best = results[0]
    print(f"\nBest gains: Kp={best[1]}, Ki={best[2]}, Kd={best[3]} (RMSE={best[0]:.2f} mm)")
    print("Update KP/KI/KD in firmware/rover_firmware.ino and simulation/rover_sim.py accordingly.")


if __name__ == "__main__":
    main()
