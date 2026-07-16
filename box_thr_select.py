# Copyright (C) 2026 Daniel Haensse
# GPL-3.0-or-later (see repo LICENSE)
"""Data-driven maneuver-throttle selection: reads the sweep flights
(work/sweep-<model>-<thr>/jsbsim_log_inverted.csv), computes each peak
altitude, and per model picks the HIGHEST throttle whose peak stays
within the video ceiling - measured, not guessed. Writes a shell-
sourceable choice file.

    python3 box_thr_select.py /workall /out/thr_choice.env
"""
import csv
import os
import sys

CEILING = 118.0
MODELS = ["turbotimber", "kingfisher", "dragonfly"]
THRS = [1150, 1250, 1350, 1450, 1550, 1650]


def peak(path):
    try:
        return max(float(r["alt"]) for r in csv.DictReader(open(path)))
    except Exception:
        return None


def main():
    workall, out = sys.argv[1], sys.argv[2]
    lines = []
    for model in MODELS:
        best = None
        report = []
        for thr in THRS:
            p = peak(f"{workall}/sweep-{model}-{thr}/jsbsim_log_inverted.csv")
            report.append(f"{thr}:{'dead' if p is None else f'{p:.0f}m'}")
            if p is not None and p <= CEILING:
                best = max(best or 0, thr)
        if best is None:
            best = THRS[0]
            report.append("NONE within ceiling -> lowest")
        print(f"{model}: {' '.join(report)} -> thr {best}", flush=True)
        lines.append(f"THR_{model}={best}")
    with open(out, "w") as fh:
        fh.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
