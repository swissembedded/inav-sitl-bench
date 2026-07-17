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


# Phases where sustained rotation makes the AHRS excursion a documented
# estimator limit (task #7 tracks fixing THAT) - the relaxed divergence
# bound applies ONLY here, not to the whole flight
SPIN_PHASES = {"spin-hold", "spin-rud", "rud-release"}


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
    # AHRS honesty, scoped: 90 deg is tolerated ONLY inside the spin
    # phases (sustained-rotation estimator limit); everywhere else 15 -
    # a blanket 90 for spin-capable repertoires masked failures in the
    # other figures
    div_spin, div_rest = 0.0, 0.0
    for r in rows:
        d = _tilt_div(float(r["fc_roll"]), float(r["fc_pitch"]),
                      float(r["js_roll"]), float(r["js_pitch"]))
        if r["phase"] in SPIN_PHASES:
            div_spin = max(div_spin, d)
        else:
            div_rest = max(div_rest, d)
    if div_rest > 15.0:
        fails.append(f"AHRS divergence {div_rest:.0f} deg > 15 (outside spin)")
    if div_spin > 90.0:
        fails.append(f"AHRS divergence {div_spin:.0f} deg > 90 (in spin)")
    for fig in repertoire:
        for want in FIG_MODE.get(fig, []):
            if not any(want in r["mode"] for r in rows):
                fails.append(f"FC never reported {want} ({fig})")
    # HELD-ATTITUDE checks on plant truth over the frames where the FC
    # reports the mode - reporting the box while flying the figure badly
    # must not pass (the old batch gate had these; the show gate lost them)
    def _mode_frames(want):
        sel = [r for r in rows if want in r["mode"]]
        return sel[len(sel) // 4:]          # skip the entry transient
    if "inverted" in repertoire:
        mid = _mode_frames("INVERT")
        if mid:
            v = statistics.median(abs(float(r["js_roll"])) for r in mid)
            if v < 150:
                fails.append(f"inverted roll median {v:.0f} < 150")
    for fig, want, sign in (("knife", "KNIFE L", -1), ("knife", "KNIFE R", +1),
                            ("knife_fast", "KNIFE L", -1), ("knife_fast", "KNIFE R", +1)):
        if fig in repertoire:
            mid = _mode_frames(want)
            if mid:
                v = statistics.median(float(r["js_roll"]) for r in mid)
                if abs(v - sign * 90) > 25:
                    fails.append(f"{want} roll median {v:.0f} not ~{sign * 90}")
    if "hang" in repertoire:
        mid = _mode_frames("P-HANG")
        if mid:
            v = statistics.median(float(r["js_pitch"]) for r in mid)
            if v < 75:
                fails.append(f"hang pitch median {v:.0f} < 75")
    if "flaps_harrier" in repertoire or "flaps_slow" in repertoire:
        if not any(float(r.get("flap", 0) or 0) > 0.8 for r in rows):
            fails.append("flaps never deployed past 0.8")
    tail = rows[-60:]
    endroll = statistics.median(abs(float(r["js_roll"])) for r in tail)
    if endroll > 30:
        fails.append(f"end roll {endroll:.0f} > 30 (not recovered)")
    return (not fails), fails


def verify_gyro_pair():
    """Automated gate for the autogyro tip-over pair (no hand review):
    the MANUAL flight must genuinely tip and end grounded, the GUARD
    flight must catch (visible throttle floor against a low stick), stay
    off the terrain, and end AIRBORNE wings-level. Returns (ok, fails)."""
    fails = []
    man = list(csv.DictReader(open("jsbsim_log_autog2_tip_manual.csv")))
    grd = list(csv.DictReader(open("jsbsim_log_autog2_tip_guard.csv")))

    def flight(rows):
        return [r for r in rows if float(r["t"]) > 28]

    m, g = flight(man), flight(grd)
    # manual: the tip-over must happen (deep roll excursion) ...
    if max(abs(float(r["js_roll"])) for r in m) < 120:
        fails.append("manual: never tipped past 120 deg")
    # ... the guard must NOT have been armed ...
    if any("GUARD" in r["mode"] for r in m):
        fails.append("manual: guard box was armed")
    # ... and it ends on the ground (the honest failure)
    if float(man[-1]["alt"]) > 3.0:
        fails.append(f"manual: ends airborne at {man[-1]['alt']} m")

    # guard: armed the whole flight, something to catch, catches visible
    if not all("GUARD" in r["mode"] for r in g):
        fails.append("guard: box not armed throughout")
    if max(abs(float(r["js_roll"])) for r in g) < 45:
        fails.append("guard: no excursion to catch (starving too weak)")
    # the catch signature: FC throttle output well above a starving stick
    catches = sum(1 for r in g
                  if float(r["st_thr"]) < 1250 and float(r["thr"]) > 0.5)
    if catches < 1000:      # >= 1 s of active throttle floor
        fails.append("guard: throttle floor never engaged against low stick")
    if min(float(r["alt"]) for r in g) < 10.0:
        fails.append(f"guard: sank to {min(float(r['alt']) for r in g):.1f} m")
    tail = grd[-500:]
    if float(grd[-1]["alt"]) < 10.0:
        fails.append(f"guard: ends at {grd[-1]['alt']} m, not airborne")
    endroll = statistics.median(abs(float(r["js_roll"])) for r in tail)
    if endroll > 20:
        fails.append(f"guard: end roll {endroll:.0f} > 20")
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
