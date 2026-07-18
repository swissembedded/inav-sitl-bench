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
    # ceiling: 122 m everywhere EXCEPT the spin block - the spin needs its
    # 60-75 m ABOVE the floor line to be its own proof (Daniel: go higher
    # if it needs the height), so its entry lives at 130 with a 140 sanity
    # cap; "transit" climbs toward it are exempted with the same cap
    SPIN_EXEMPT = SPIN_PHASES | {"transit-spin", "base-spin"}
    peak_rest = max((float(r["alt"]) for r in rows
                     if r["phase"] not in SPIN_EXEMPT), default=0.0)
    if peak_rest > 122.0:
        fails.append(f"peak {peak_rest:.0f} m > 122 (outside spin)")
    # 148 = spin entry target 130 + the worst measured climb overshoot on
    # a T/W-2 airframe (12 m) + margin; the cap exists to catch runaways
    # (the 161 m finale zoom class), not approach overshoot
    if max(alts) > 148.0:
        fails.append(f"peak {max(alts):.0f} m > 148 (spin sanity cap)")
    if min(alts) < 15.0:
        fails.append(f"floor missed: min {min(alts):.0f} m < 15")
    # FLOOR-LINE honesty (needs the FW safety column, logs before it skip):
    # the show arms at 25 m true, the floor line sits at home + 30 m = 55 m
    # true. The FW engages on BREAKING THROUGH the line sinking (no
    # prediction - Daniel: a piloted trajectory is not predictable), so:
    # (a) the floor must actually ARM during the initial climb (the pt17
    # class of silent never-armed flights), (b) below the line while
    # sinking the recovery must be engaged, (c) figures fly above the
    # line and must show essentially no recovery override.
    if rows and "safety" in rows[0]:
        FLOOR_LINE = 25.0 + 30.0
        armed_frames = sum(1 for r in rows if int(r["safety"]) & 1)
        if not armed_frames:
            fails.append("floor never ARMED (safety word bit0)")
        # breach = below the line while SINKING - the initial climb passes
        # the line upward with the floor correctly quiet (first gate
        # version forgot the sink condition and failed every climb-out)
        breach = []
        prev_alt = None
        for r in rows:
            a = float(r["alt"])
            sinking = prev_alt is not None and a < prev_alt
            prev_alt = a
            if (a < FLOOR_LINE - 5 and sinking and r["phase"] not in
                    ("settle", "cal", "armL", "armH", "level", "einflug")):
                breach.append(r)
        missed = sum(1 for r in breach if not int(r["safety"]) & 2)
        if breach and missed > len(breach) * 0.2:
            fails.append(f"below the floor line without recovery in "
                         f"{missed}/{len(breach)} breach frames")
        # figures must be THEIR OWN proof: essentially no floor override
        # inside figure frames (the finale is the floor's stage instead)
        # rud-release is the spin's RECOVERY tail: the predictive floor
        # correctly primes there while the sink is arrested (measured:
        # 0 override in spin-hold/spin-rud, all of it in rud-release) -
        # the held figure is judged, not its recovery
        fig_frames = [r for r in rows if r["phase"] not in
                      ("settle", "cal", "armL", "armH", "level", "einflug",
                       "transit", "transit-spin", "transit-floor", "base",
                       "base-spin", "base-floor", "bleed", "exit",
                       "rud-release", "floor-dive", "caught")]
        overridden = sum(1 for r in fig_frames if int(r["safety"]) & 2)
        if fig_frames and overridden > len(fig_frames) * 0.05:
            fails.append(f"figures flown under floor override "
                         f"{overridden}/{len(fig_frames)} frames - not the "
                         f"figure's own proof")
        # THE FINALE: the held dive must be CAUGHT - recovery engaged during
        # floor-dive/caught, and the flight ends flying (end-level is
        # checked globally below)
        finale = [r for r in rows if r["phase"] in ("floor-dive", "caught")]
        if finale:
            if not any(int(r["safety"]) & 2 for r in finale):
                fails.append("finale: floor never caught the held dive")
    # AHRS honesty: 15 deg everywhere, 20 in the spin phases - the
    # "sustained-rotation known limit" that justified a 90 deg allowance
    # dissolved under measurement: the estimator tracks flat/inverted
    # spins at 2-6 deg (acc weight correctly drops to zero at spin
    # rates), and the historical 50+ deg outliers were the PLANT
    # teleporting (FDM discontinuity), not the AHRS. Plant teleports are
    # flagged as their own failure: a truth-attitude step no aircraft
    # can fly invalidates the flight physically.
    div_spin, div_rest = 0.0, 0.0
    prev_up = None
    teleport = None
    for r in rows:
        d = _tilt_div(float(r["fc_roll"]), float(r["fc_pitch"]),
                      float(r["js_roll"]), float(r["js_pitch"]))
        if r["phase"] in SPIN_PHASES:
            div_spin = max(div_spin, d)
        else:
            div_rest = max(div_rest, d)
        step = _tilt_div(float(r["js_roll"]), float(r["js_pitch"]),
                         *prev_up) if prev_up else 0.0
        if step > 30.0 and teleport is None:
            teleport = f"t={r['t']} ({r['phase']})"
        prev_up = (float(r["js_roll"]), float(r["js_pitch"]))
    if teleport:
        fails.append(f"plant discontinuity (FDM artifact) at {teleport} - "
                     f"truth attitude stepped >30 deg between frames")
    if div_rest > 15.0:
        fails.append(f"AHRS divergence {div_rest:.0f} deg > 15 (outside spin)")
    if div_spin > 20.0:
        fails.append(f"AHRS divergence {div_spin:.0f} deg > 20 (in spin)")
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
    # ... and it goes IN: ground contact after the deep excursion. The
    # wreck physics blow up numerically afterwards (contact springs ramp
    # the altitude to 9e15), so the END of the log is meaningless - the
    # honest check is that the tip led to the ground, not what the
    # exploded numbers do after impact
    tipped = next((i for i, r in enumerate(m)
                   if abs(float(r["js_roll"])) > 120), None)
    if tipped is not None and not any(float(r["alt"]) < 1.0
                                      for r in m[tipped:]):
        fails.append("manual: tipped but never hit the ground")

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
