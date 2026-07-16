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

"""Airframe figure reels: fly a plant model through the SAME figure set -
loop, roll, inverted, knife edge, spin - with a simple scripted P-attitude
pilot on plant truth (no FC). The two STOL trainers add their party piece,
the slow-flap harrier, which also exercises the flap bar in the replay.
Writes the jsbsim_fly CSV format (plus a 'flap' column) so
animate_jsbsim.py renders directly:

    python demo_reel.py [turbotimber kingfisher dragonfly]
    python animate_jsbsim.py demo_<model> --title "..."

All reels respect the 120 m video ceiling.
"""
import csv
import math
import sys

from jsbsim_plant import JSBSimPlant

DT = 0.01


class Pilot:
    """P-attitude pilot on plant truth. Enough to fly demo figures;
    deliberately no I-terms, no estimator - the star is the airframe."""

    def __init__(self, kp_r=0.05, kd_r=0.010, kp_p=0.06, kd_p=0.012):
        self.kp_r, self.kd_r = kp_r, kd_r
        self.kp_p, self.kd_p = kp_p, kd_p

    def step(self, p, roll_t, pitch_t):
        roll, pitch, _ = p.rpy()
        pr = math.degrees(p.fdm["velocities/p-rad_sec"])
        qr = math.degrees(p.fdm["velocities/q-rad_sec"])
        err_r = (roll_t - roll + 540.0) % 360.0 - 180.0
        ail = self.kp_r * err_r - self.kd_r * pr
        err_p = pitch_t - pitch
        # inverted: the body-frame elevator sense flips with the roll
        upright = math.cos(math.radians(roll)) >= 0.0
        ele = -(self.kp_p * err_p - self.kd_p * qr) * (1.0 if upright else -1.0)
        return max(-1, min(1, ail)), max(-1, min(1, ele))


_TRACK = {"roll": "velocities/p-rad_sec", "pitch": "velocities/q-rad_sec",
          "yaw": "velocities/r-rad_sec"}


def fly(model, phases, alt_ft, kts):
    p = JSBSimPlant(model=model, alt_ft=alt_ft, kts=kts, dt=DT)
    pilot = Pilot()
    log = open(f"jsbsim_log_demo_{model}.csv", "w", newline="")
    w = csv.writer(log)
    w.writerow("t,phase,mode,fc_roll,fc_pitch,fc_yaw,js_roll,js_pitch,js_yaw,"
               "ias,alt,ail,ele,rud,thr,fc_thr,st_ail,st_ele,st_thr,st_rud,"
               "st_arm,st_angle,st_inv,st_sel,fc_alt,tvc_p,tvc_y,x,y,gps_fix,"
               "gps_sat,flap".split(","))
    t = 0.0
    flap_cur = 0.0
    peak = 0.0
    for ph in phases:
        state = {"rot": 0.0}
        t_start = t
        while True:
            el = t - t_start
            done = (el >= ph["dur"]) if "dur" in ph else ph["done"](p, state, el)
            if done:
                break
            roll_t = ph.get("roll_t", 0.0)
            pitch_t = ph.get("pitch_t", 0.0)
            thr = ph.get("thr", 0.5)
            if "alt_hold" in ph:
                # scene-local throttle governor (a water hang IS constant
                # pilot throttle work): base + P on height + D on sink
                vz = (p.z - state.get("z_prev", p.z)) / DT
                state["z_prev"] = p.z
                err = ph["alt_hold"] - p.z
                thr = max(0.0, min(1.0, thr
                                   + max(-0.12, min(0.35, 0.2 * err))
                                   - 0.35 * vz))
            ail, ele = pilot.step(p, roll_t, pitch_t)
            if "ail_ovr" in ph:
                ail = ph["ail_ovr"]
            if "ele_ovr" in ph:
                ele = ph["ele_ovr"]
            rud = ph.get("rud", 0.0)
            # pilot-side flap ramp: the 2 s smix-style deployment
            f_t = ph.get("flaps", flap_cur)
            rate = DT / ph.get("flap_ramp_s", 2.0)
            flap_cur += max(-rate, min(rate, f_t - flap_cur))
            p.set_flaps(flap_cur)
            p.set_controls(ail, ele, rud, thr)
            p.step(DT)
            state["rot"] += math.degrees(
                p.fdm[_TRACK[ph.get("track", "pitch")]]) * DT
            r, pit, yw = p.rpy()
            xy = p.xy()
            peak = max(peak, p.z)
            w.writerow([f"{t:.2f}", ph["name"], ph.get("mode", "DEMO"),
                        f"{r:.1f}", f"{pit:.1f}", f"{yw:.1f}",
                        f"{r:.1f}", f"{pit:.1f}", f"{yw:.1f}",
                        f"{p.ias_kts():.0f}", f"{p.z:.1f}",
                        f"{ail:.2f}", f"{ele:.2f}", f"{rud:.2f}",
                        f"{thr:.2f}", f"{thr:.2f}",
                        f"{1500 + ail * 500:.0f}", f"{1500 + ele * 500:.0f}",
                        f"{1000 + thr * 1000:.0f}", f"{1500 + rud * 500:.0f}",
                        "2000", "1500", "1500", "1500",
                        f"{p.z:.1f}", "0", "0",
                        f"{xy[0]:.1f}", f"{xy[1]:.1f}", "0", "0",
                        f"{flap_cur:.2f}"])
            t += DT
        print(f"  {ph['name']:12s} done at t={t:5.1f}s  alt {p.z:5.1f} m  "
              f"ias {p.ias_kts() * 0.51444:4.1f} m/s", flush=True)
    log.close()
    print(f"  peak altitude {peak:.1f} m", flush=True)


def rot_done(deg, tmax=8.0):
    return lambda p, s, el: abs(s["rot"]) >= deg or el > tmax


def slow_done(v_ms, tmax=8.0):
    return lambda p, s, el: p.ias_kts() * 0.51444 <= v_ms or el > tmax


def figures(cruise_thr, loop_ele, roll_ail, invert_pitch, invert_thr,
            knife_rud, knife_thr, spin_ias, spin_thr,
            climb_thr=0.9, climb_pitch=14.0, cruise_pitch=0.0,
            climb_s=4.0, inv_s=3.5, knife_s=3.0, loop_thr=1.0):
    """The common figure set: loop, roll, inverted, knife, spin. Sinking
    figures are separated by climb legs (like real display flying), so the
    whole reel lives inside the 0..120 m video window."""
    climb = lambda dur: dict(name="climb", dur=dur, pitch_t=climb_pitch,
                             thr=climb_thr)
    return [
        dict(name="level", dur=3.0, thr=cruise_thr, pitch_t=cruise_pitch),
        dict(name="loop", done=rot_done(350), ele_ovr=loop_ele, thr=loop_thr,
             track="pitch", mode="LOOP"),
        dict(name="level", dur=2.0, thr=cruise_thr, pitch_t=cruise_pitch),
        # the spin flies FIRST while the altitude is still up there - it is
        # the hungriest figure, and the weak climbers cannot buy the height
        # back mid-reel
        # entry: clean nose-up deceleration under the pilot (an ele
        # override would mush for seconds and eat the spin altitude)
        dict(name="spin-entry", done=slow_done(spin_ias, 5.0), pitch_t=16.0,
             thr=spin_thr, mode="SPIN"),
        dict(name="spin", done=rot_done(900, 9.0), ele_ovr=-0.5, rud=1.0,
             thr=spin_thr, track="yaw", mode="SPIN"),
        dict(name="spin-exit", dur=0.6, ele_ovr=0.1, rud=-0.4, thr=spin_thr,
             mode="SPIN"),
        dict(name="recover", dur=2.0, pitch_t=0.0,
             thr=min(1.0, cruise_thr + 0.15)),
        climb(climb_s),
        dict(name="roll", done=rot_done(350), ail_ovr=roll_ail,
             thr=min(1.0, cruise_thr + 0.15), track="roll", mode="ROLL"),
        climb(climb_s),
        dict(name="invert", dur=inv_s, roll_t=180.0, pitch_t=invert_pitch,
             thr=invert_thr, mode="INVERTED"),
        climb(climb_s + 1.0),
        dict(name="knife", dur=knife_s, roll_t=90.0, rud=knife_rud, thr=knife_thr,
             mode="KNIFE"),
        dict(name="recover", dur=2.5, pitch_t=6.0,
             thr=min(1.0, cruise_thr + 0.25)),
    ]


def flap_show(thr, harrier_pitch, harrier_thr, hang_pitch, hang_thr):
    """STOL party piece: 2 s flap deployment, harrier, then the HANG -
    nose high on flaps and prop, walking speed. This is what the flaps
    are for, and why the slow smix deployment matters."""
    return [
        dict(name="flaps-out", dur=3.0, flaps=1.0, pitch_t=5.0, thr=thr,
             mode="FLAPS 2s"),
        dict(name="harrier", dur=4.0, flaps=1.0, pitch_t=harrier_pitch,
             thr=harrier_thr, mode="FLAPS"),
        dict(name="hang", dur=8.0, flaps=1.0, pitch_t=hang_pitch,
             thr=hang_thr, mode="FLAPS HANG"),
        dict(name="flaps-in", dur=3.0, flaps=0.0, pitch_t=2.0, thr=thr,
             mode="FLAPS 2s"),
    ]


MODELS = {
    "turbotimber": dict(alt_ft=279, kts=12 / 0.51444, phases=(
        figures(cruise_thr=0.15, loop_ele=-0.45, roll_ail=0.6,
                invert_pitch=10.0, invert_thr=0.15,
                knife_rud=-0.85, knife_thr=0.35, spin_ias=8.0, spin_thr=0.06,
                climb_thr=0.35, loop_thr=0.6)
        + [dict(name="descend", dur=5.5, pitch_t=-8.0, thr=0.05)]
        + flap_show(thr=0.15, harrier_pitch=25.0, harrier_thr=0.24,
                    hang_pitch=60.0, hang_thr=0.34)[:-1]
        + [dict(name="punch", dur=1.5, flaps=1.0, pitch_t=80.0, thr=1.0,
                mode="PUNCH"),
           dict(name="flaps-in", dur=3.0, flaps=0.0, pitch_t=2.0, thr=0.15,
                mode="FLAPS 2s"),
           dict(name="level", dur=2.5, thr=0.15, pitch_t=-2.0)])),
    "kingfisher": dict(alt_ft=10, kts=8 / 0.51444, phases=(
        # Act 1, a metre over the water: the ground-effect cushion carries
        # what free air will refuse at the end of the reel
        [dict(name="level", dur=1.5, thr=0.15),
         dict(name="flaps-out", dur=3.5, flaps=1.0, pitch_t=3.0, thr=0.2,
              mode="FLAPS 2s"),
         dict(name="water-hang", dur=11.0, flaps=1.0, pitch_t=76.0,
              thr=0.75, alt_hold=1.2, mode="IGE HANG"),
         dict(name="climb-out", dur=3.0, flaps=1.0, pitch_t=12.0, thr=1.0),
         dict(name="flaps-in", dur=3.0, flaps=0.0, pitch_t=8.0, thr=1.0,
              mode="FLAPS 2s"),
         dict(name="climb", dur=9.0, pitch_t=9.0, thr=1.0)]
        # Act 2, the figures up high
        + figures(cruise_thr=0.5, loop_ele=-0.42, roll_ail=0.6,
                  invert_pitch=12.0, invert_thr=0.6,
                  knife_rud=-0.85, knife_thr=1.0, spin_ias=9.5, spin_thr=0.12,
                  climb_thr=0.95, climb_pitch=8.0, climb_s=5.0, inv_s=2.8,
                  knife_s=2.5)
        # Act 3, the contrast: nose toward the hover, throttle pegged -
        # free air settles anyway. T/W 0.78 only hangs on the cushion.
        + [dict(name="flaps-out", dur=2.5, flaps=1.0, pitch_t=8.0, thr=0.6,
                mode="FLAPS 2s"),
           dict(name="harrier", dur=4.5, flaps=1.0, pitch_t=22.0, thr=0.65,
                mode="FLAPS"),
           dict(name="hover-try", dur=12.0, flaps=1.0, pitch_t=76.0, thr=1.0,
                mode="NO HOVER"),
           dict(name="recover", dur=2.5, pitch_t=2.0, thr=1.0),
           dict(name="flaps-in", dur=3.0, flaps=0.0, pitch_t=2.0, thr=0.55,
                mode="FLAPS 2s"),
           dict(name="level", dur=2.5, thr=0.55)])),
    "dragonfly": dict(alt_ft=213, kts=16 / 0.51444, phases=(
        figures(cruise_thr=0.55, loop_ele=-0.5, roll_ail=0.8,
                invert_pitch=8.0, invert_thr=0.7,
                knife_rud=-0.85, knife_thr=1.0, spin_ias=9.0, spin_thr=0.15,
                climb_thr=0.75, climb_pitch=8.0, cruise_pitch=-3.0,
                climb_s=2.5)
        + [dict(name="level", dur=2.5, thr=0.55, pitch_t=-3.0)])),
}

if __name__ == "__main__":
    which = [a for a in sys.argv[1:] if a in MODELS] or list(MODELS)
    for model in which:
        cfg = MODELS[model]
        print(f"=== {model} ===", flush=True)
        fly(model, cfg["phases"], cfg["alt_ft"], cfg["kts"])
