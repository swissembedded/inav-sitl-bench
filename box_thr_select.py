# Copyright (C) 2026 Daniel Haensse
# GPL-3.0-or-later (see repo LICENSE)
"""Data-driven throttle-scale selection: reads the sweep flights
(work/sweep-<model>-<scale>/jsbsim_log_inverted.csv), computes each peak
altitude, and per model picks the HIGHEST scale whose peak stays within
the video ceiling - measured, not guessed. The scale multiplies EVERY
phase's throttle in jsbsim_fly (--thr-scale), so the aerobat3d-calibrated
choreography flies proportionally on a different power loading. Writes a
shell-sourceable choice file.

    python3 box_thr_select.py /workall /out/thr_choice.env
"""
import csv
import sys

CEILING = 118.0
MODELS = ["turbotimber", "kingfisher", "dragonfly"]
SCALES = ["0.5", "0.65", "0.8", "1.0"]


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
        for sc in SCALES:
            p = peak(f"{workall}/sweep-{model}-{sc}/jsbsim_log_inverted.csv")
            report.append(f"x{sc}:{'dead' if p is None else f'{p:.0f}m'}")
            if p is not None and p <= CEILING:
                best = sc if best is None else max(best, sc, key=float)
        if best is None:
            best = SCALES[0]
            report.append("NONE within ceiling -> lowest")
        print(f"{model}: {' '.join(report)} -> scale {best}", flush=True)
        lines.append(f"SCALE_{model}={best}")
    with open(out, "w") as fh:
        fh.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
