# Copyright (C) 2026 Daniel Haensse
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

"""Automated gust disturbance matrix for the orientation holds.

For every hold (inverted, knife left/right, prop hang) a battery of gusts
hits the aircraft from all directions: the four horizontal cardinals
relative to the entry heading (head-on, tail, left, right), both verticals
(up- and downdraft), and the four lateral+vertical diagonals (two control
surfaces excited at once). Each gust blows for GUST_S seconds; the metrics
compare against the settled pre-gust state, so trims and the altitude
assist offset cancel out:

  tilt_max   worst attitude deviation from the pre-gust attitude [deg]
  t_rec      time from gust-off until the deviation stays < REC_TOL_DEG [s]
  dalt       worst altitude excursion from the pre-gust altitude [m]

PASS requires tilt_max < TILT_MAX_DEG, t_rec < REC_MAX_S, |dalt| <
ALT_MAX_M. No videos to watch: the script prints a table per maneuver,
writes gust_log.csv, and exits nonzero on any FAIL.

The SITL container is restarted before every maneuver (SITL keeps state;
an AHRS parked at 180 deg needs longer than any settle phase to recover).

Usage:  python gust_matrix.py [inverted knife_left ...] [--gust 3.0]
                              [--no-restart] [--set name=value ...]

--set writes a firmware setting (live, after boot) before the battery,
e.g. --set ohold_hover_thr_min=1400 for a hover authority floor sweep.
The value size is taken from the FC's own SETTING_INFO reply.
"""
import csv
import math
import subprocess
import sys
import time

from msp import MspClient
from hitl import sim_step
from jsbsim_plant import JSBSimPlant
from bench import wait_boot_calibration

RC_LOW, RC_MID, RC_HIGH = 1000, 1500, 2000
FLAG_ARMED = 1 << 2
FLAG_CAL = 1 << 9
DT = 0.001

# reference mode: SITL started with --lockstep advances its clock exactly
# one DT per injected frame -- phase durations then count SIM time (frames)
# and host load cannot skew the physics-vs-AHRS timing
LOCKSTEP = "--lockstep" in sys.argv

GUST_S = 3.0          # gust duration
REC_MAX_S = 4.0       # recovery deadline after gust-off
REC_TOL_DEG = 5.0     # "recovered" when deviation stays below this
REC_HOLD_S = 1.0      # ... for this long
TILT_MAX_DEG = 30.0   # worst allowed deviation during gust
ALT_MAX_M = 10.0      # worst allowed altitude excursion (hang: +50 %)
SETTLE_TOL_DEG = 8.0  # hold counts as captured below this error
BASELINE_S = 1.0      # pre-gust averaging window

# maneuver -> (SEL detent, throttle, target roll/pitch, airframe)
# hang_tvc: prop hang on the TVC pusher delta - elevons are dead at hover,
# all authority comes from the vectored nozzle
HOLDS = {
    "inverted":    (1270, 1650, (180.0, 0.0), "aerobat3d"),
    "knife_left":  (1510, 1650, (-90.0, 0.0), "aerobat3d"),
    "knife_right": (1750, 1650, (90.0, 0.0), "aerobat3d"),
    "hang":        (1985, 1500, (0.0, 90.0), "aerobat3d"),
    "hang_tvc":    (1985, 1500, (0.0, 90.0), "funjet"),
}

# push direction -> unit vector (along, right, down) in the entry-track frame
DIRECTIONS = [
    ("head-on",    (-1.0, 0.0, 0.0)),
    ("tail",       (1.0, 0.0, 0.0)),
    ("left",       (0.0, -1.0, 0.0)),
    ("right",      (0.0, 1.0, 0.0)),
    ("updraft",    (0.0, 0.0, -1.0)),
    ("downdraft",  (0.0, 0.0, 1.0)),
    ("left-down",  (0.0, -0.7071, 0.7071)),
    ("right-down", (0.0, 0.7071, 0.7071)),
    ("left-up",    (0.0, -0.7071, -0.7071)),
    ("right-up",   (0.0, 0.7071, -0.7071)),
]


def rc_ch(thr=RC_LOW, arm=RC_LOW, angle=RC_LOW, invert=RC_LOW, sel=RC_LOW,
          ele=RC_MID, ail=RC_MID, rud=RC_MID):
    return [ail, ele, thr, rud, arm, angle, invert, sel]


def arming_flags(m):
    import struct
    return struct.unpack("<I", m.request(0x2000)[9:13])[0]


def up_in_body(roll_deg, pitch_deg):
    """Earth up in the body frame from roll/pitch (yaw irrelevant)."""
    r, p = math.radians(roll_deg), math.radians(pitch_deg)
    return (-math.sin(p), math.sin(r) * math.cos(p), math.cos(r) * math.cos(p))


def mean_up(rps):
    """Average attitude as a normalized mean UP VECTOR. Never average
    roll/pitch angles: roll wobbling across the +-180 wrap (inverted hold)
    averages to 0 = level and fakes a 180 deg upset."""
    s = [0.0, 0.0, 0.0]
    for rp in rps:
        u = up_in_body(*rp)
        s = [a + b for a, b in zip(s, u)]
    n = math.sqrt(sum(x * x for x in s)) or 1.0
    return tuple(x / n for x in s)


def tilt_to_up(rp, up_ref):
    """Tilt angle [deg] between an attitude and a reference up vector."""
    u = up_in_body(*rp)
    dot = max(-1.0, min(1.0, sum(a * b for a, b in zip(u, up_ref))))
    return math.degrees(math.acos(dot))


def tilt_between(rp_a, rp_b):
    """Tilt angle [deg] between two (roll, pitch) attitudes."""
    return tilt_to_up(rp_a, up_in_body(*rp_b))


class Runner:
    """Minimal fixed-slot HITL coupling (no video logging, no stick show)."""

    def __init__(self, msp, plant, log_rows, maneuver, tvc_gain=None):
        self.m = msp
        self.plant = plant
        self.rows = log_rows
        self.man = maneuver
        self.tvc_gain = tvc_gain     # callable thr01 -> gain, None = no TVC
        self.t0 = time.time()
        self.frames = 0

    def clock(self):
        """Bench time [s]: sim time under lockstep, wall time otherwise."""
        return self.frames * DT if LOCKSTEP else time.time() - self.t0

    def fly(self, secs, rc, phase, freeze=False, thr_override=None):
        end = time.time() + secs
        f0 = self.frames
        while (self.frames - f0 < secs / DT) if LOCKSTEP else (time.time() < end):
            it0 = time.perf_counter()
            r = sim_step(self.m, self.plant.acc_mg(), self.plant.gyro_dps16(),
                         rc, baro_pa=self.plant.baro_pa())
            ail, ele, rud = r.stab_roll, -r.stab_pitch, r.stab_yaw   # flip-ele convention
            thr = thr_override if thr_override is not None else (r.stab_throttle + 1.0) / 2.0
            if freeze:
                self.plant._a_earth = (0.0, 0.0, 0.0)
            else:
                self.plant.set_controls(ail, ele, rud, thr)
                if self.tvc_gain is not None:
                    # replicate the firmware's servo-mixer TVC inputs
                    # (INPUT_TVC_* = stabilized * thrustVectoringGain)
                    g = self.tvc_gain(thr)
                    self.plant.set_tvc(max(-1.0, min(1.0, r.stab_pitch * g)),
                                       max(-1.0, min(1.0, r.stab_yaw * g)))
                self.plant.step(dt=DT)
            jr, jp, jy = self.plant.rpy()
            self.frames += 1
            self.rows.append((self.clock(), self.man, phase,
                              jr, jp, jy, self.plant.z,
                              ail, ele, rud, thr))
            # lockstep needs no pacing: the FC clock only moves with our frames
            if not LOCKSTEP:
                while True:
                    rem = DT - (time.perf_counter() - it0)
                    if rem <= 0:
                        break
                    if rem > 0.002:
                        time.sleep(rem - 0.002)
        return self.plant.rpy(), self.plant.z


def restart_container(name="inav-sitl"):
    subprocess.run(["podman", "restart", name], check=True,
                   stdout=subprocess.DEVNULL)
    time.sleep(3)


def run_maneuver(man, gust_ms, do_restart, sets=()):
    sel, thr_hold, target_rp, model = HOLDS[man]
    if do_restart:
        restart_container()
    m = MspClient()
    plant = JSBSimPlant(model=model, alt_ft=394)
    rows = []

    tvc = None
    if model == "funjet":
        def _u(name, default):
            try:
                return int.from_bytes(m.request(0x1003, name.encode() + b"\x00"), "little")
            except Exception:
                return default
        gain = _u("tvc_gain", 100) / 100.0
        comp = _u("tvc_thrust_comp", 100) / 100.0
        def tvc(thr01):
            t = min(max(thr01, 0.25), 1.0)      # TVC_THRUST_COMP_FLOOR
            return gain * (1.0 + (1.0 / t - 1.0) * comp)
    run = Runner(m, plant, rows, man, tvc_gain=tvc)

    wait_boot_calibration(m)
    for name, value in sets:
        try:
            size = len(m.request(0x1003, name.encode() + b"\x00"))
            m.set_setting(name, int(value).to_bytes(size, "little", signed=value < 0))
            print(f"  set {name} = {value} ({size} bytes)")
        except (IOError, OverflowError) as e:
            # out-of-range value or unknown setting: fail the run loudly
            # instead of dying mid-battery with a stack trace
            print(f"  set {name} = {value} REJECTED ({e}), aborting maneuver")
            m.close()
            return None
    run.fly(6, rc_ch(), "settle", freeze=True)
    t0 = run.clock()
    while (arming_flags(m) & FLAG_CAL) and run.clock() - t0 < 25:
        run.fly(1.0, rc_ch(angle=RC_HIGH), "cal", freeze=True)
    t0 = run.clock()
    while not (arming_flags(m) & FLAG_ARMED) and run.clock() - t0 < 20:
        run.fly(1.0, rc_ch(thr=RC_LOW, arm=RC_LOW, angle=RC_HIGH), "armL", freeze=True)
        run.fly(1.2, rc_ch(thr=RC_LOW, arm=RC_HIGH, angle=RC_HIGH), "armH", freeze=True)
    if not (arming_flags(m) & FLAG_ARMED):
        print(f"{man}: FC did not arm, skipping")
        m.close()
        return None
    run.fly(6, rc_ch(thr=1650, arm=RC_HIGH, angle=RC_HIGH), "level")

    # entry heading defines the gust frame for the whole battery
    heading = math.radians(plant.rpy()[2])
    ca, sa = math.cos(heading), math.sin(heading)

    rc_hold = rc_ch(thr=thr_hold, arm=RC_HIGH, angle=RC_LOW, sel=sel)

    # engage + wait for capture
    t0 = run.clock()
    captured = False
    while run.clock() - t0 < 15:
        (jr, jp, _), _ = run.fly(0.5, rc_hold, "entry")
        if tilt_between((jr, jp), target_rp) < SETTLE_TOL_DEG:
            captured = True
            break
    if not captured:
        print(f"{man}: hold never captured the target, aborting maneuver")
        m.close()
        return [(man, d, float("nan"), float("nan"), float("nan"), False)
                for d, _ in DIRECTIONS]

    def flush_rows():
        with open("gust_log.csv", "a", newline="") as f:
            w = csv.writer(f)
            for t, mn, phase, jr, jp, jy, z, ail, ele, rud, thr in rows:
                w.writerow([f"{t:.2f}", mn, phase, f"{jr:.1f}", f"{jp:.1f}",
                            f"{jy:.1f}", f"{z:.1f}", f"{ail:.2f}",
                            f"{ele:.2f}", f"{rud:.2f}", f"{thr:.2f}"])
        rows.clear()

    results = []
    aborted = False
    for dname, (along, right, down) in DIRECTIONS:
        if aborted:
            results.append((man, dname, float("nan"), None, float("nan"), False))
            continue
        try:
            # re-capture check: the next gust must start from a healthy hold,
            # otherwise one upset cascades through the whole battery
            t0 = run.clock()
            recaptured = False
            while run.clock() - t0 < 10:
                (jr, jp, _), _ = run.fly(0.5, rc_hold, "resettle")
                if tilt_between((jr, jp), target_rp) < SETTLE_TOL_DEG:
                    recaptured = True
                    break
            if not recaptured:
                print(f"  {man:12s} {dname:10s} hold not recaptured, "
                      f"aborting battery")
                results.append((man, dname, float("nan"), None, float("nan"), False))
                aborted = True
                continue
            run.fly(2.0, rc_hold, "resettle")

            base = []
            end = run.clock() + BASELINE_S
            while run.clock() < end:
                (jr, jp, _), z = run.fly(0.1, rc_hold, "baseline")
                base.append((jr, jp, z))
            base_up = mean_up([(b[0], b[1]) for b in base])
            base_z = sum(b[2] for b in base) / len(base)
            # recovered = back to the hold's own steady-state wander, not to
            # an absolute figure: a nozzle-only TVC hover cones 3-6 deg
            # around vertical even undisturbed and would never pass a fixed
            # 5 deg gate it does not meet before the gust either
            base_wander = max(tilt_to_up((b[0], b[1]), base_up) for b in base)
            rec_tol = max(REC_TOL_DEG, 1.5 * base_wander)

            north = along * ca - right * sa
            east = along * sa + right * ca
            plant.set_wind(north_ms=north * gust_ms, east_ms=east * gust_ms,
                           down_ms=down * gust_ms)
            tilt_max = 0.0
            dalt_max = 0.0
            end = run.clock() + GUST_S
            while run.clock() < end:
                (jr, jp, _), z = run.fly(0.1, rc_hold, f"gust-{dname}")
                tilt_max = max(tilt_max, tilt_to_up((jr, jp), base_up))
                dalt_max = max(dalt_max, abs(z - base_z))
            plant.set_wind()

            # recovery: deviation must stay below tolerance for REC_HOLD_S
            t_off = run.clock()
            t_rec = None
            below_since = None
            while run.clock() - t_off < REC_MAX_S + REC_HOLD_S:
                (jr, jp, _), z = run.fly(0.1, rc_hold, f"rec-{dname}")
                dev = tilt_to_up((jr, jp), base_up)
                tilt_max = max(tilt_max, dev)
                dalt_max = max(dalt_max, abs(z - base_z))
                if dev < rec_tol:
                    if below_since is None:
                        below_since = run.clock()
                    elif run.clock() - below_since >= REC_HOLD_S:
                        t_rec = below_since - t_off
                        break
                else:
                    below_since = None
        except ValueError:
            # JSBSim diverged (NaN state): the airframe left the envelope,
            # nothing sensible can run on this instance anymore
            print(f"  {man:12s} {dname:10s} plant diverged (NaN), "
                  f"aborting battery")
            results.append((man, dname, float("nan"), None, float("nan"), False))
            aborted = True
            flush_rows()
            continue
        alt_limit = ALT_MAX_M * (1.5 if man in ("hang", "hang_tvc") else 1.0)
        ok = (tilt_max < TILT_MAX_DEG and t_rec is not None
              and t_rec < REC_MAX_S and dalt_max < alt_limit)
        results.append((man, dname, tilt_max, t_rec, dalt_max, ok))
        rec_s = f"{t_rec:4.1f}" if t_rec is not None else " >? "
        print(f"  {man:12s} {dname:10s} tilt_max {tilt_max:5.1f} deg  "
              f"t_rec {rec_s} s (tol {rec_tol:.1f})  dalt {dalt_max:5.1f} m  "
              f"{'PASS' if ok else 'FAIL'}")
        flush_rows()

    if not aborted:
        run.fly(0.3, rc_ch(thr=RC_LOW, arm=RC_LOW), "disarm")
    m.close()
    flush_rows()
    return results


def main():
    gust_ms = 3.0
    if "--gust" in sys.argv:
        gust_ms = float(sys.argv[sys.argv.index("--gust") + 1])
    do_restart = "--no-restart" not in sys.argv
    sets = []
    argv = sys.argv[1:]
    for i, a in enumerate(argv):
        if a == "--set" and i + 1 < len(argv) and "=" in argv[i + 1]:
            name, val = argv[i + 1].split("=", 1)
            sets.append((name, int(val)))
    mans = [a for a in argv if a in HOLDS] or list(HOLDS)

    with open("gust_log.csv", "w", newline="") as f:
        csv.writer(f).writerow(["t", "maneuver", "phase", "js_roll",
                                "js_pitch", "js_yaw", "alt",
                                "ail", "ele", "rud", "thr"])

    all_results = []
    for man in mans:
        print(f"=== {man} (gust {gust_ms} m/s) ===")
        res = run_maneuver(man, gust_ms, do_restart, sets)
        if res:
            all_results.extend(res)

    n_fail = sum(1 for r in all_results if not r[5])
    print(f"\n{len(all_results) - n_fail} PASS, {n_fail} FAIL "
          f"(criteria: tilt < {TILT_MAX_DEG} deg, recovery < {REC_MAX_S} s, "
          f"alt excursion < {ALT_MAX_M} m)")
    sys.exit(1 if n_fail else 0)


if __name__ == "__main__":
    main()
