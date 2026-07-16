# Copyright (C) 2026 Daniel Haensse
# GPL-3.0-or-later (see repo LICENSE)
"""ONE VIDEO PER AIRPLANE: for each model the FC is provisioned with the
actuator-true mixer (airframe_provision), the capability-derived `show`
sequence is flown under the real FC (Einflug first, floor always on),
the automated gate judges the log, and only a PASS renders.

    python _run_show_videos.py [model ...]        (default: whole hangar)
"""
import csv
import os
import shutil
import statistics
import subprocess
import sys

from airframe_config import AIRFRAMES

CONTAINER = os.environ.get("INAV_SITL_CONTAINER", "inav-sitl")

# FC mode the readback must report per repertoire figure (None = no box)
FIG_MODE = {"inverted": ["INVERT"], "roll": ["F ROLL"], "loop": ["F LOOP"],
            "knife": ["KNIFE L", "KNIFE R"], "knife_fast": ["KNIFE L", "KNIFE R"],
            "spin": ["FLAT SPIN"], "hang": ["P-HANG"],
            "flaps_harrier": [], "flaps_slow": []}


def run(*a, **kw):
    subprocess.run(a, check=kw.pop("check", True), **kw)


def _tilt_div(fr, fp, jr, jp):
    import math
    def up(r, p):
        r, p = math.radians(r), math.radians(p)
        return (-math.sin(p), math.sin(r) * math.cos(p),
                math.cos(r) * math.cos(p))
    a, b = up(fr, fp), up(jr, jp)
    return math.degrees(math.acos(max(-1.0, min(1.0,
        sum(x * y for x, y in zip(a, b))))))


def verify_show(tag, repertoire):
    rows = list(csv.DictReader(open(f"jsbsim_log_{tag}.csv")))
    fails = []
    alts = [float(r["alt"]) for r in rows]
    crash = next((i for i, a in enumerate(alts) if a < 0.0), None)
    if crash is not None:
        return False, [f"crashed into terrain at t={rows[crash]['t']}s "
                       f"({rows[crash]['phase']})"]
    if max(alts) > 122.0:
        fails.append(f"peak {max(alts):.0f} m > 122")
    if min(alts) < 15.0:
        fails.append(f"floor missed: min {min(alts):.0f} m < 15")
    div = max(_tilt_div(float(r["fc_roll"]), float(r["fc_pitch"]),
                        float(r["js_roll"]), float(r["js_pitch"]))
              for r in rows)
    limit = 90.0 if "spin" in repertoire else 15.0
    if div > limit:
        fails.append(f"AHRS divergence {div:.0f} deg > {limit:.0f}")
    for fig in repertoire:
        for want in FIG_MODE.get(fig, []):
            if not any(want in r["mode"] for r in rows):
                fails.append(f"FC never reported {want} ({fig})")
    tail = rows[-60:]
    endroll = statistics.median(abs(float(r["js_roll"])) for r in tail)
    if endroll > 30:
        fails.append(f"end roll {endroll:.0f} > 30 (not recovered)")
    return (not fails), fails


def main():
    which = [a for a in sys.argv[1:] if a in AIRFRAMES] or [
        m for m in AIRFRAMES if AIRFRAMES[m][0] != "GYRO"]
    results = []
    for model in which:
        actuators, repertoire = AIRFRAMES[model]
        if actuators == "GYRO":
            print(f"=== {model}: gyro flies its own pair, skipped ===")
            continue
        print(f"=== {model} show ===", flush=True)
        if os.path.exists("fcdata/eeprom.bin"):
            os.remove("fcdata/eeprom.bin")
        run("podman", "restart", CONTAINER)
        run(sys.executable, "-c", "import time; time.sleep(4)")
        run(sys.executable, "airframe_provision.py", model)
        # the flight aborts loudly when the FC will not arm (boot race,
        # observed once) - one restart + retry covers it
        for attempt in (1, 2):
            run("podman", "restart", CONTAINER)
            run(sys.executable, "-c", "import time; time.sleep(4)")
            r = subprocess.run([sys.executable, "jsbsim_fly.py", "--flip-ele",
                                "--lockstep", "--model", model, "show"])
            if r.returncode == 0:
                break
            print(f"  flight aborted (attempt {attempt}), retrying", flush=True)
        tag = f"{model}_show"
        shutil.copy("jsbsim_log_show.csv", f"jsbsim_log_{tag}.csv")
        if os.path.exists("jsbsim_params_show.txt"):
            shutil.copy("jsbsim_params_show.txt", f"jsbsim_params_{tag}.txt")
        ok, fails = verify_show(tag, repertoire)
        results.append((tag, ok, fails))
        if ok:
            run(sys.executable, "animate_jsbsim.py", tag, "--title",
                f"{model} under the real FC: {', '.join(repertoire)}")
        print(f"=== {tag}: {'PASS' if ok else 'FAIL ' + '; '.join(fails)} ===",
              flush=True)
    print("\n=== SUMMARY ===", flush=True)
    for tag, ok, fails in results:
        print(f"  {tag:24s} {'PASS' if ok else 'FAIL: ' + '; '.join(fails)}",
              flush=True)
    npass = sum(1 for _, ok, _ in results if ok)
    print(f"{npass}/{len(results)} PASS (only PASS rendered)", flush=True)
    sys.exit(0 if npass == len(results) else 1)


if __name__ == "__main__":
    main()
