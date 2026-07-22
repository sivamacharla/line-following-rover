"""
Grid-search PID tuning against the rover simulation.

Sweeps Kp/Ki/Kd combinations, scores each by trajectory RMSE against the
track, and reports the best-performing gains — mirroring the iterative,
data-driven tuning process described in the project writeup.

Each gain set is evaluated over multiple random seeds (not just one run):
sensor noise means a single run can get lucky or unlucky, so we score on
mean RMSE and treat worst-case RMSE as a stability gate. A gain set that
looks great on one noise realization but diverges on another is rejected.
"""

import itertools
import random

from rover_sim import SimConfig, run_simulation, rmse_from_track

KP_RANGE = [12.0, 18.0, 22.0, 28.0, 34.0]
KI_RANGE = [0.0, 0.3, 0.6, 1.0]
KD_RANGE = [4.0, 7.0, 9.5, 13.0]

SEEDS = list(range(8))          # noise realizations per gain set
DIVERGENCE_THRESHOLD_MM = 30.0  # a run this far off track counts as a failure


def evaluate(kp, ki, kd):
    scores = []
    for seed in SEEDS:
        random.seed(seed)
        cfg = SimConfig(kp=kp, ki=ki, kd=kd)
        log = run_simulation(cfg)
        scores.append(rmse_from_track(log))
    mean_score = sum(scores) / len(scores)
    worst_score = max(scores)
    failures = sum(1 for s in scores if s > DIVERGENCE_THRESHOLD_MM)
    return mean_score, worst_score, failures


def main():
    results = []
    combos = list(itertools.product(KP_RANGE, KI_RANGE, KD_RANGE))
    print(f"Evaluating {len(combos)} PID combinations x {len(SEEDS)} seeds each...")

    for kp, ki, kd in combos:
        mean_score, worst_score, failures = evaluate(kp, ki, kd)
        results.append((mean_score, worst_score, failures, kp, ki, kd))

    # Stable gain sets first (zero divergent runs), ranked by mean RMSE;
    # unstable sets sorted after, so a bad-but-lucky mean never wins.
    results.sort(key=lambda r: (r[2] > 0, r[0]))

    print(f"\nTop 10 gain sets (0 failures = stable across all {len(SEEDS)} seeds):")
    print(f"{'MeanRMSE':>10} {'WorstRMSE':>10} {'Fails':>6} {'Kp':>8} {'Ki':>8} {'Kd':>8}")
    for mean_score, worst_score, failures, kp, ki, kd in results[:10]:
        print(f"{mean_score:10.2f} {worst_score:10.2f} {failures:6d} {kp:8.2f} {ki:8.2f} {kd:8.2f}")

    best = results[0]
    if best[2] > 0:
        print(f"\nWarning: even the best gain set diverged on {best[2]}/{len(SEEDS)} seeds.")
    print(f"\nBest gains: Kp={best[3]}, Ki={best[4]}, Kd={best[5]} "
          f"(mean RMSE={best[0]:.2f} mm, worst={best[1]:.2f} mm, failures={best[2]}/{len(SEEDS)})")
    print("Update KP/KI/KD in firmware/rover_firmware.ino and simulation/rover_sim.py accordingly.")


if __name__ == "__main__":
    main()
