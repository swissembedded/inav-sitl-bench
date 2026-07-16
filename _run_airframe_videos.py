"""FC-flown figure videos for the non-aerobat airframes: for each
(model, maneuver) the FC is provisioned fresh, the standard maneuver
script drives the switches, the FC's mode readback lands in the CSV,
and the replay is rendered as jsbsim_<model>_<maneuver>.mp4.

    python _run_airframe_videos.py [model ...]     (default: all three)

Per-model maneuver throttle (--thr) compensates the power difference to
the aerobat3d the default bands were trimmed on.
"""
import csv
import os
import shutil
import subprocess
import sys

CONTAINER = os.environ.get("INAV_SITL_CONTAINER", "inav-sitl")

# the FULL maneuver repertoire on every airframe - the gate decides what
# each one can do; an honest FAIL (dragonfly prop hang, kingfisher hover)
# is as informative as a PASS
MANEUVERS = ["inverted", "inverted_stick", "knife_left", "knife_right",
             "hang", "loop_fig", "roll_hold", "flat_spin", "inv_spin",
             "knife_spin", "snap_neg", "floor_dive", "floor_panic",
             "floor_spin"]
MODELS = {
    # model: maneuver-throttle override [us] (None = aerobat3d default)
    "turbotimber": 1300,
    "kingfisher": None,
    "dragonfly": 1750,
}
TITLE = {
    "inverted": "inverted hold through a gust and a rudder turn",
    "inverted_stick": "stick carving around the inverted reference",
    "knife_left": "knife edge hold left - the altitude assist works the nose angle",
    "knife_right": "knife edge hold right",
    "hang": "90 deg prop hang - the controller finds its own hover throttle",
    "loop_fig": "full loop at fig_loop_rate, closing on the entry altitude",
    "roll_hold": "F ROLL: commanded roll figure, level hold after",
    "flat_spin": "FLAT SPIN box: held autorotation, clean recovery on release",
    "inv_spin": "inverted flat spin: FLAT SPIN + INVERTED",
    "knife_spin": "knife edge spin: FLAT SPIN + KNIFE L",
    "snap_neg": "hard negative snap and recovery",
    "floor_dive": "altitude floor as a low safety net under a dive",
    "floor_panic": "floor catch against held down-elevator and chopped throttle",
    "floor_spin": "floor recovery out of a held spin",
}


def run(*a, **kw):
    subprocess.run(a, check=kw.pop("check", True), **kw)


# Automated per-flight gate (no hand review): every criterion is checked
# from the flight log; a flight only counts as PASS when all hold.
EXPECT_MODE = {
    "inverted": "INVERT", "inverted_stick": "INVERT",
    "knife_left": "KNIFE L", "knife_right": "KNIFE R",
    "hang": "P-HANG", "loop_fig": "F LOOP", "roll_hold": "F ROLL",
    "flat_spin": "FLAT SPIN",
    # the mode readback lists the attitude box first, masking FLAT SPIN
    "inv_spin": "INVERT", "knife_spin": "KNIFE L",
    "snap_neg": None,                       # manual figure, no box
    "floor_dive": "+FLOOR", "floor_panic": "+FLOOR", "floor_spin": "+FLOOR",
}


def _tilt_div(fr, fp, jr, jp):
    import math
    def up(r, p):
        r, p = math.radians(r), math.radians(p)
        return (-math.sin(p), math.sin(r) * math.cos(p),
                math.cos(r) * math.cos(p))
    a, b = up(fr, fp), up(jr, jp)
    return math.degrees(math.acos(max(-1.0, min(1.0,
        sum(x * y for x, y in zip(a, b))))))


def verify(tag, man):
    """Returns (ok, list of failure strings)."""
    import statistics
    rows = list(csv.DictReader(open(f"jsbsim_log_{tag}.csv")))
    fails = []
    alts = [float(r["alt"]) for r in rows]
    if max(alts) > 122.0:
        fails.append(f"peak {max(alts):.0f} m > 122")
    floor_limit = 15.0 if man.startswith("floor") else 5.0
    if min(alts) < floor_limit:
        fails.append(f"floor {min(alts):.0f} m < {floor_limit:.0f}")
    # AHRS honesty: FC estimate vs plant truth (sustained-spin excursions
    # are a documented known limit -> relaxed bound there)
    div = max(_tilt_div(float(r["fc_roll"]), float(r["fc_pitch"]),
                        float(r["js_roll"]), float(r["js_pitch"]))
              for r in rows)
    limit = 90.0 if "spin" in man else 15.0
    if div > limit:
        fails.append(f"AHRS divergence {div:.0f} deg > {limit:.0f}")
    # the FC must actually REPORT the commanded mode during the maneuver
    want = EXPECT_MODE[man]
    if want and not any(want in r["mode"] for r in rows):
        fails.append(f"FC never reported {want}")
    # held attitude, judged on plant truth in the frames where the FC
    # reports the mode active
    hold = [r for r in rows if want and want in r["mode"]]
    if hold:
        mid = hold[len(hold) // 4:]
        if man == "inverted":
            v = statistics.median(abs(float(r["js_roll"])) for r in mid)
            if v < 150:
                fails.append(f"inverted roll median {v:.0f} < 150")
        elif man == "hang":
            v = statistics.median(float(r["js_pitch"]) for r in mid)
            if v < 75:
                fails.append(f"hang pitch median {v:.0f} < 75")
        elif man in ("knife_left", "knife_right"):
            v = statistics.median(float(r["js_roll"]) for r in mid)
            if abs(abs(v) - 90) > 25:
                fails.append(f"knife roll median {v:.0f} not ~90")
    # figures/spins/floor/snap must end recovered and level; the HOLDS end
    # in the held attitude by design (see jsbsim_fly _EXPECT)
    if man not in ("inverted", "inverted_stick", "knife_left", "knife_right"):
        tail = rows[-60:]
        endroll = statistics.median(abs(float(r["js_roll"])) for r in tail)
        if endroll > 30:
            fails.append(f"end roll {endroll:.0f} > 30 (not recovered)")
    return (not fails), fails


def main():
    which = [a for a in sys.argv[1:] if a in MODELS] or list(MODELS)
    results = []
    for model in which:
        thr = MODELS[model]
        for man in MANEUVERS:
            print(f"=== {model} / {man} ===", flush=True)
            if os.path.exists("fcdata/eeprom.bin"):
                os.remove("fcdata/eeprom.bin")
            run("podman", "restart", CONTAINER)
            run(sys.executable, "-c", "import time; time.sleep(3)")
            run(sys.executable, "bench.py", "provision")
            run("podman", "restart", CONTAINER)
            run(sys.executable, "-c", "import time; time.sleep(3)")
            cmd = [sys.executable, "jsbsim_fly.py", "--flip-ele",
                   "--lockstep", "--model", model, man]
            if thr is not None:
                cmd += ["--thr", str(thr)]
            run(*cmd)
            tag = f"{model}_{man}"
            shutil.copy(f"jsbsim_log_{man}.csv", f"jsbsim_log_{tag}.csv")
            if os.path.exists(f"jsbsim_params_{man}.txt"):
                shutil.copy(f"jsbsim_params_{man}.txt",
                            f"jsbsim_params_{tag}.txt")
            ok, fails = verify(tag, man)
            results.append((tag, ok, fails))
            if ok:
                run(sys.executable, "animate_jsbsim.py", tag, "--title",
                    f"{model} under the real FC: {TITLE.get(man, man)}")
            print(f"=== {tag}: {'PASS' if ok else 'FAIL ' + '; '.join(fails)} ===",
                  flush=True)
    print("\n=== SUMMARY ===", flush=True)
    for tag, ok, fails in results:
        print(f"  {tag:28s} {'PASS' if ok else 'FAIL: ' + '; '.join(fails)}",
              flush=True)
    npass = sum(1 for _, ok, _ in results if ok)
    print(f"{npass}/{len(results)} PASS (only PASS flights were rendered)",
          flush=True)
    sys.exit(0 if npass == len(results) else 1)


if __name__ == "__main__":
    main()
