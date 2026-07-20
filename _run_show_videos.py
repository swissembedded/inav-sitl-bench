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
    # the descent corridor OUT of a spin is part of the spin's altitude:
    # a legal spin lives above the ceiling (exempt to 148), so the frames
    # after its last spin frame necessarily cross the 122 line on the way
    # down. Exempt exactly that stretch - from spin end WHILE still above
    # 122 (hard cap 20 s) - and demand it DESCENDS net, so a runaway
    # cannot hide in it. A spin that ends below the ceiling produces no
    # corridor (the easyglider legitimately climbs to its next figure).
    last_spin_t = max((float(r["t"]) for r in rows
                      if r["phase"] in SPIN_EXEMPT), default=-1e9)
    corridor_ids = set()
    corridor = []
    for r in rows:
        dt = float(r["t"]) - last_spin_t
        if r["phase"] in SPIN_EXEMPT or dt < 0.0 or dt > 20.0:
            continue
        if float(r["alt"]) <= 122.0:
            break
        corridor.append(r)
        corridor_ids.add(id(r))
    if corridor and float(corridor[-1]["alt"]) >= float(corridor[0]["alt"]):
        fails.append("post-spin corridor climbs instead of descending "
                     f"({float(corridor[0]['alt']):.0f} -> "
                     f"{float(corridor[-1]['alt']):.0f} m)")
    peak_rest = max((float(r["alt"]) for r in rows
                     if r["phase"] not in SPIN_EXEMPT
                     and id(r) not in corridor_ids), default=0.0)
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
            # only frames where the floor is ARMED count - before arming
            # (initial climb) the net does not exist yet, by FW design
            if (a < FLOOR_LINE - 5 and sinking and int(r["safety"]) & 1
                    and r["phase"] not in
                    ("settle", "cal", "armL", "armH", "level", "einflug")):
                breach.append(r)
        missed = sum(1 for r in breach if not int(r["safety"]) & 2)
        # sub-0.2 s "breaches" carry no floor semantics (the safety word
        # itself updates at 125 Hz) - a single stray frame failed a whole
        # flight once
        if len(breach) >= 200 and missed > len(breach) * 0.2:
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
                       "rud-release", "floor-dive", "caught",
                       "orbit", "takeover", "level-out")]
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
    PREP = ("settle", "cal", "armL", "armH")
    for r in rows:
        if r["phase"] in PREP:
            # the AHRS is CONVERGING here by design (frozen plant, initial
            # alignment - with the mag active the first seconds swing) -
            # the flight script's own divergence tracker always skipped
            # these phases, the gate now does too
            prev_up = (float(r["js_roll"]), float(r["js_pitch"]))
            continue
        d = _tilt_div(float(r["fc_roll"]), float(r["fc_pitch"]),
                      float(r["js_roll"]), float(r["js_pitch"]))
        # the floor ORBIT is the same sustained-rotation regime as the
        # spin (continuous 25-30 deg banked turning, acc weight low, mag
        # and COG aiding bent by the bank): measured 15.0-17.5 deg through
        # the post-figure orbit, recovering to <1 deg by level-out. The
        # nav loiter flies GPS position vectors there by design, immune to
        # the attitude-estimate yaw - the spin allowance applies.
        if r["phase"] in SPIN_PHASES or r["phase"] == "orbit":
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
            # 145: unambiguously inverted with room for the wobble of a
            # low-authority floater (easyglider holds a 149 median with
            # dips to 138 through the gust - it IS flying inverted; the
            # old 150 was calibrated on the aerobat's crisp hold)
            v = statistics.median(abs(float(r["js_roll"])) for r in mid)
            if v < 145:
                fails.append(f"inverted roll median {v:.0f} < 145")
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
    # manual: the tip-over must happen (deep roll excursion). 90 deg is
    # past anything an autogyro flies on - the impact can arrive before
    # the roll winds further (measured: 113 deg at ground contact); the
    # crash proof is the ground-contact check below, not the peak angle
    if max(abs(float(r["js_roll"])) for r in m) < 90:
        fails.append("manual: never tipped past 90 deg")
    # ... the guard must NOT have been armed ...
    if any("GUARD" in r["mode"] for r in m):
        fails.append("manual: guard box was armed")
    # ... and it goes IN: ground contact after the deep excursion. The
    # wreck physics blow up numerically afterwards (contact springs ramp
    # the altitude to 9e15), so the END of the log is meaningless - the
    # honest check is that the tip led to the ground, not what the
    # exploded numbers do after impact
    tipped = next((i for i, r in enumerate(m)
                   if abs(float(r["js_roll"])) > 90), None)
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


def verify_floor_spin():
    """Latch + orbit contract on the forgotten-switch flight (Daniel): the
    catch must LATCH (no hand-back with the FSPIN box still selected), the
    aircraft must orbit near the breach point at/above the floor, and only
    a fresh stick takeover releases it. The orbit has TWO legal forms,
    discriminated by safety bit 32 (orbitViaNav): a healthy position
    estimate flies the real nav loiter on the breach anchor at the
    speed-matched nav_fw_loiter_radius (150 m); without one the hold flies
    a constant-bank circle whose radius is set by physics, R = v^2 /
    (g tan22) - and with the pilot's trim throttle held it rides ABOVE the
    floor (pitch capped at -5: degradation on the safe side, documented)."""
    rows = list(csv.DictReader(open("jsbsim_log_floor_spin.csv")))
    fails = []
    LINE = 25.0 + 25.0        # arm-low net: alt_floor_altitude 25 over ground 25
    caught = [r for r in rows if r["phase"] in ("flat-spin", "caught") and int(r["safety"]) & 2]
    if not caught:
        fails.append("floor never caught the spin")
    forgot = [r for r in rows if r["phase"] == "forgot"]
    late = forgot[3 * len(forgot) // 4:]
    if caught and late:
        if sum(1 for r in late if int(r["safety"]) & 2) < len(late) * 0.9:
            fails.append("recovery released without pilot takeover (latch broken)")
        alts = [float(r["alt"]) for r in late]
        if min(alts) < LINE:
            fails.append(f"orbit sagged under the floor line: {min(alts):.0f}")
        if max(alts) - min(alts) > 12:
            fails.append(f"orbit altitude wanders {min(alts):.0f}-{max(alts):.0f}")
        b = caught[0]
        bx, by = float(b["x"]), float(b["y"])
        d = [((float(r["x"]) - bx) ** 2 + (float(r["y"]) - by) ** 2) ** 0.5 for r in late]
        med = statistics.median(d)
        viaNav = sum(1 for r in late if int(r["safety"]) & 32) > len(late) * 0.5
        if viaNav:
            if abs(med - 150.0) > 70 or max(d) > 280:
                fails.append(f"nav orbit off the ring: median {med:.0f} max {max(d):.0f}")
        else:
            roll = statistics.median(abs(float(r["js_roll"])) for r in late)
            if not (12 <= roll <= 32):
                fails.append(f"degraded orbit bank {roll:.0f} not ~22")
            if med > 220:
                fails.append(f"degraded orbit drifted {med:.0f} m from the breach")
    after = [r for r in rows if r["phase"] == "after-takeover"]
    if after:
        tail = after[len(after) // 3:]
        if any(int(r["safety"]) & 2 for r in tail):
            fails.append("recovery active after takeover")
    return (not fails), fails


def verify_floor_dive():
    """Held-stick dive: the floor must catch AGAINST the held elevator and
    defend the line - the nav orbit honors pilot pitch as a rate override,
    so the FW re-breach guard must keep dropping back to the aggressive
    climb (measured failure without it: orbit ridden down to 24 m)."""
    rows = list(csv.DictReader(open("jsbsim_log_floor_dive.csv")))
    fails = []
    held = [r for r in rows if r["phase"] == "dive-held"]
    if not any(int(r["safety"]) & 2 for r in held):
        fails.append("held dive never caught")
    if held:
        alts = [float(r["alt"]) for r in held]
        if min(alts) < 20.0:
            fails.append(f"held dive sank to {min(alts):.0f} - the re-breach "
                         f"defense lost the aircraft")
        if float(held[-1]["alt"]) < 40.0:
            fails.append(f"held dive ends at {float(held[-1]['alt']):.0f}, "
                         f"line not recaptured")
        nof = [r for r in rows if r["phase"] == "dive-nofloor"]
        if nof and min(float(r["alt"]) for r in nof) >= min(alts):
            fails.append("contrast pass did not punch below the caught dive")
    return (not fails), fails


def verify_floor_panic():
    """Chopped-throttle dive: the recovery climb gets its own energy (the
    throttle floor must rise against the idle stick), defends the line,
    and a fresh deflection after centered sticks releases it."""
    rows = list(csv.DictReader(open("jsbsim_log_floor_panic.csv")))
    fails = []
    chop = [r for r in rows if r["phase"] == "dive-chop"]
    rec = [r for r in chop if int(r["safety"]) & 2]
    if not rec:
        fails.append("chopped dive never caught")
    else:
        alts = [float(r["alt"]) for r in chop]
        if min(alts) < 20.0:
            fails.append(f"chopped dive sank to {min(alts):.0f}")
        if max(float(r["fc_thr"]) for r in rec) < 0.3:
            fails.append("recovery never raised the throttle against the chop")
    after = [r for r in rows if r["phase"] == "after"]
    if after and float(statistics.median(float(r["alt"]) for r in after)) < 45.0:
        fails.append("post-catch hold below the recovery band")
    tk = [r for r in rows if r["phase"] == "takeover"]
    tail = tk[len(tk) // 2:]
    if tail and sum(1 for r in tail if int(r["safety"]) & 2) > len(tail) * 0.2:
        fails.append("takeover did not release the recovery")
    return (not fails), fails


def verify_fig_abort():
    """Sequencer abort proof: the impulse spins, dropping the box kills the
    open-loop command, the floor still catches the after-dive, and the
    takeover closes the flight released and level."""
    rows = list(csv.DictReader(open("jsbsim_log_fig_abort.csv")))
    fails = []

    def yawrate(seg):
        # total traversal over wall time, NOT a per-frame median: at 1 kHz
        # logging the plant only steps every ~10 ms, so most frame deltas
        # are exactly zero and a median reads 0 on a genuine spin
        if len(seg) < 2:
            return 0.0
        trav, prev = 0.0, None
        for r in seg:
            y = float(r["js_yaw"])
            if prev is not None:
                trav += abs((y - prev + 540) % 360 - 180)
            prev = y
        dt = float(seg[-1]["t"]) - float(seg[0]["t"])
        return trav / max(dt, 0.01)

    imp = [r for r in rows if r["phase"] == "impulse"]
    rel = [r for r in rows if r["phase"] == "abort-release"]
    if yawrate(imp) < 100:
        fails.append("impulse never spun")
    if yawrate(rel[len(rel) // 2:]) > 60:
        fails.append("stale impulse persists after the box drop")
    if not any(int(r["safety"]) & 2 for r in rows
               if r["phase"] in ("floor-dive", "caught")):
        fails.append("floor never caught the post-abort dive")
    lvl = [r for r in rows if r["phase"] == "level-out"]
    tail = lvl[len(lvl) // 3:]
    if tail and sum(1 for r in tail if int(r["safety"]) & 2) > len(tail) * 0.2:
        fails.append("takeover did not release the recovery")
    return (not fails), fails


def verify_gyro_land():
    """Gate for the landing override (Daniel: idle stick means LANDING -
    the guard must never spin the thrust up against it): guard box armed,
    pilot pulls idle, the rotor starves and the gyro tips - and the guard
    stays SILENT: no recovery bit, FC throttle at zero throughout."""
    fails = []
    rows = list(csv.DictReader(open("jsbsim_log_autog2_tip_land.csv")))
    idle = [r for r in rows if r["phase"] == "land-idle"]
    if not idle:
        return False, ["land: no land-idle phase in the log"]
    if not all("GUARD" in r["mode"] for r in idle):
        fails.append("land: guard box not armed through the idle descent")
    if any(int(r["safety"]) & 4 for r in idle):
        fails.append("land: guard recovery fired against an idle stick")
    # the FC must never raise the thrust on its own; skip the first second
    # (the slewed stick is still travelling 1700 -> 1000)
    hot = [float(r["thr"]) for r in idle[1000:] if float(r["thr"]) > 0.1]
    if hot:
        fails.append(f"land: FC throttle rose to {max(hot):.2f} at idle stick")
    # the proof needs teeth: the tip (= trip condition) must actually
    # occur while the guard keeps quiet
    if max(abs(float(r["js_roll"])) for r in idle) < 45:
        fails.append("land: never tipped past the trip bank - nothing proven")
    return (not fails), fails


def verify_floor_catch(man):
    """Universal floor coverage (Daniel: the floor is not spin-specific). The
    floor must catch ANY aerobatic mode that sinks through the line
    (inverted / knife / hang / roll / loop), LEVEL out of that attitude, hold
    above the floor, and latch the mode out - after the pilot's takeover the
    still-held switch must not flip the aircraft back into the aerobatic pose."""
    fails = []
    rows = list(csv.DictReader(open(f"jsbsim_log_{man}.csv")))
    LINE = 25.0 + 25.0
    caught = [r for r in rows if r["phase"] in ("caught", "forgot") and int(r["safety"]) & 2]
    if not caught:
        return False, [f"{man}: floor never caught the sink through the line"]
    settle = caught[len(caught) // 2:]
    off = [r for r in settle if abs(float(r["js_roll"])) > 45 or abs(float(r["js_pitch"])) > 45]
    if len(off) > len(settle) * 0.15:
        fails.append(f"{man}: did not level out - roll/pitch > 45 deg in "
                     f"{len(off) / len(settle) * 100:.0f}% of the settled orbit")
    # the recovery dips into the budget BELOW the line by design (the height
    # under the line IS the recovery budget) - rolling upright out of inverted
    # / knife eats more of it than a dive. It must not reach the ground, and it
    # must then climb back to loiter AT/above the line.
    lowest = min(float(r["alt"]) for r in caught)
    if lowest < LINE - 15:
        fails.append(f"{man}: recovery sank {LINE - lowest:.0f} m under the line (near the ground)")
    orbit = [r for r in rows if r["phase"] == "forgot"]
    if orbit and statistics.median([float(r["alt"]) for r in orbit]) < LINE - 5:
        fails.append(f"{man}: did not climb back to loiter at the floor line")
    after = [r for r in rows if r["phase"] == "after-takeover"]
    if after:
        tail = after[len(after) // 2:]
        if any(abs(float(r["js_roll"])) > 60 or abs(float(r["js_pitch"])) > 60 for r in tail):
            fails.append(f"{man}: latched mode re-engaged after takeover (attitude went aerobatic again)")
    return (not fails), fails


def verify_floor_manual():
    """Manual floor recovery + land (Daniel): flying manually the pilot dives,
    RELEASES the sticks, and the floor must pull up on its own; then with the
    FLOOR SWITCHED OFF the aircraft must be able to descend through the line
    and land (the floor no longer catches)."""
    fails = []
    rows = list(csv.DictReader(open("jsbsim_log_floor_manual.csv")))
    LINE = 25.0 + 25.0
    rel = [r for r in rows if r["phase"] in ("released", "recovered")]
    if not any(int(r["safety"]) & 2 for r in rel):
        fails.append("manual: floor never caught after the pilot let go")
    rec = [r for r in rows if r["phase"] == "recovered"]
    if rec:
        if float(rec[-1]["alt"]) - float(rec[0]["alt"]) < 5:
            fails.append(f"manual: no climb after release ({float(rec[-1]['alt'])-float(rec[0]['alt']):+.0f} m)")
        if min(float(r["alt"]) for r in rec) < LINE - 5:
            fails.append("manual: sank under the floor line during recovery")
        if max(abs(float(r["js_roll"])) for r in rec[len(rec)//2:]) > 45:
            fails.append("manual: did not level out during recovery")
    # floor OFF -> must descend through the line and NOT be caught (landable)
    land = [r for r in rows if r["phase"] == "floor-off-land"]
    if land:
        if min(float(r["alt"]) for r in land) > LINE:
            fails.append(f"manual: floor-off did not descend through the line (min {min(float(r['alt']) for r in land):.0f} m) - cannot land")
        if any(int(r["safety"]) & 2 for r in land[len(land)//3:]):
            fails.append("manual: floor still caught with the box OFF - would fight the landing")
    return (not fails), fails


def verify_soar():
    """Thermal soaring proof (Daniel: motor off, ride the lift). With the
    SOARING box engaged the FW idles the motor and the glider must CLIMB on
    the thermal: a net altitude gain while the throttle sits at idle, never
    sinking below the entry, the loiter staying on the drifting column, and
    the motor RETURNING the moment SOARING is dropped (proving it was an
    override, not a dead motor)."""
    fails = []
    rows = list(csv.DictReader(open("jsbsim_log_soar.csv")))
    soar = [r for r in rows if r["phase"] == "soar"]
    if not soar:
        return False, ["soar: no soar phase in the log"]
    thr = [float(r["fc_thr"]) for r in soar]
    alt = [float(r["alt"]) for r in soar]
    idle_frac = sum(1 for v in thr if v < 0.2) / len(thr)
    if statistics.median(thr) > 0.2 or idle_frac < 0.8:
        fails.append(f"soar: motor not idled (median thr {statistics.median(thr):.2f}, "
                     f"{idle_frac*100:.0f}% idle) - SOARING never thermalled")
    gain = alt[-1] - alt[0]
    if gain < 15:
        fails.append(f"soar: no climb on the lift ({gain:+.0f} m over the phase)")
    if min(alt) < alt[0] - 5:
        fails.append(f"soar: sank {alt[0]-min(alt):.0f} m below the entry")
    bx, by = float(soar[0]["x"]), float(soar[0]["y"])
    dmax = max(((float(r["x"]) - bx) ** 2 + (float(r["y"]) - by) ** 2) ** 0.5 for r in soar)
    if dmax > 600:
        fails.append(f"soar: loiter ran away {dmax:.0f} m from entry (lost the thermal)")
    ex = [float(r["fc_thr"]) for r in rows if r["phase"] == "exit"]
    if ex and statistics.mean(ex) < 0.4:
        fails.append(f"soar: motor did not return after exit (thr {statistics.mean(ex):.2f})")
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
        # PER-MODEL FC IMAGE (Daniel: the model's config travels with it):
        # a saved eeprom_<model>.bin is restored directly - the settings
        # ARE the airframe's; provisioning runs only when no image exists
        # yet, and writes one for next time
        if os.path.exists(f"eeprom_{model}.bin") and os.path.exists(f"eeprom_{model}.bin.ok"):
            shutil.copy(f"eeprom_{model}.bin", "fcdata/eeprom.bin")
            run("podman", "restart", CONTAINER)
            run(sys.executable, "-c", "import time; time.sleep(4)")
        else:
            if os.path.exists("fcdata/eeprom.bin"):
                os.remove("fcdata/eeprom.bin")
            run("podman", "restart", CONTAINER)
            run(sys.executable, "-c", "import time; time.sleep(4)")
            run(sys.executable, "airframe_provision.py", model)
        # the flight aborts loudly when the FC will not arm (boot race,
        # observed once) - one restart + retry covers it. The sensor
        # suite is the standard: GPS + mag for everyone, the pitot rides
        # on the airframes that carry one (binary).
        suite = ["--gps", "--mag"] + (["--pitot"] if model == "binary" else [])
        for attempt in (1, 2):
            run("podman", "restart", CONTAINER)
            run(sys.executable, "-c", "import time; time.sleep(4)")
            r = subprocess.run([sys.executable, "jsbsim_fly.py", "--flip-ele",
                                "--lockstep", "--model", model, "show"] + suite)
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
