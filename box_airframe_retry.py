# Copyright (C) 2026 Daniel Haensse
# GPL-3.0-or-later (see repo LICENSE)
"""Data-driven start-altitude correction: for every non-floor flight that
busted the video ceiling, compute a lowered start from the MEASURED
overshoot (new start = old start - (peak - 115)) and emit retry lines

    <model> <maneuver> <new_start_m>

for the batch script. Floor demos arm low by design - their ceiling comes
from the climb-out and is the throttle scale's job, not the start's.

    python3 box_airframe_retry.py /workall
"""
import csv
import sys

CEILING = 122.0
TARGET = 115.0
MODELS = ["turbotimber", "kingfisher", "dragonfly",
          "easyglider", "easystar", "xeno", "aeroscout", "a10", "icona5", "bf109",
          "lippisch", "mig15", "pt17", "binary", "arwing", "deltastrike", "vampire"]
MANEUVERS = ["inverted", "inverted_stick", "knife_left", "knife_right",
             "hang", "loop_fig", "roll_hold", "flat_spin", "inv_spin",
             "knife_spin", "snap_neg"]


def main():
    workall = sys.argv[1]
    for model in MODELS:
        for man in MANEUVERS:
            path = f"{workall}/af-{model}-{man}/jsbsim_log_{man}.csv"
            try:
                rows = list(csv.DictReader(open(path)))
                alts = [float(r["alt"]) for r in rows]
            except Exception:
                continue
            pk = max(alts)
            if pk <= CEILING or pk > 100000:   # blowups: no start fixes those
                continue
            start = alts[0]
            new_start = max(20.0, start - (pk - TARGET))
            print(f"{model} {man} {new_start:.0f}")


if __name__ == "__main__":
    main()
