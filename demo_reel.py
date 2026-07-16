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

"""Airframe demo reels: fly a plant model through the maneuvers the
AIRFRAME can do (and, deliberately, one it cannot) with a simple scripted
P-attitude pilot on plant truth - no FC in the loop. Writes the flight in
the jsbsim_fly CSV format so animate_jsbsim.py renders it directly:

    python demo_reel.py turbotimber
    python animate_jsbsim.py demo_turbotimber --title "..."

The point of the reels is the AIRFRAME character: the turbotimber's
slotted-flap harrier, the kingfisher's tamer plain-flap version, and the
dragonfly pusher demonstrating loops/rolls at speed plus the honest
hang-attempt fall-through (no propwash authority, thrust line above CG).
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


def fly(model, phases, alt_ft, kts):
    p = JSBSimPlant(model=model, alt_ft=alt_ft, kts=kts, dt=DT)
    pilot = Pilot()
    log = open(f"jsbsim_log_demo_{model}.csv", "w", newline="")
    w = csv.writer(log)
    w.writerow("t,phase,mode,fc_roll,fc_pitch,fc_yaw,js_roll,js_pitch,js_yaw,"
               "ias,alt,ail,ele,rud,thr,fc_thr,st_ail,st_ele,st_thr,st_rud,"
               "st_arm,st_angle,st_inv,st_sel,fc_alt,tvc_p,tvc_y,x,y,gps_fix,"
               "gps_sat".split(","))
    t = 0.0
    flap_cur = 0.0
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
                (p.fdm["velocities/p-rad_sec"] if ph.get("track") == "roll"
                 else p.fdm["velocities/q-rad_sec"])) * DT
            r, pit, yw = p.rpy()
            xy = p.xy()
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
                        f"{xy[0]:.1f}", f"{xy[1]:.1f}", "0", "0"])
            t += DT
        print(f"  {ph['name']:12s} done at t={t:5.1f}s  alt {p.z:5.1f} m  "
              f"ias {p.ias_kts() * 0.51444:4.1f} m/s", flush=True)
    log.close()


def rot_done(deg):
    return lambda p, s, el: abs(s["rot"]) >= deg or el > 8.0


MODELS = {
    "turbotimber": dict(alt_ft=492, kts=12 / 0.51444, phases=[
        dict(name="level", dur=3.0, thr=0.45),
        dict(name="loop", done=rot_done(350), ele_ovr=-0.45, thr=1.0, track="pitch"),
        dict(name="level", dur=2.5, thr=0.45),
        dict(name="roll", done=rot_done(350), ail_ovr=0.6, thr=0.6, track="roll"),
        dict(name="level", dur=2.0, thr=0.45),
        dict(name="invert", dur=4.0, roll_t=180.0, pitch_t=10.0, thr=0.55),
        dict(name="level", dur=2.0, thr=0.45),
        dict(name="flaps-out", dur=3.0, flaps=1.0, pitch_t=8.0, thr=0.5,
             mode="FLAPS 2s"),
        dict(name="harrier", dur=6.0, flaps=1.0, pitch_t=25.0, thr=0.62,
             mode="FLAPS"),
        dict(name="flaps-in", dur=3.0, flaps=0.0, pitch_t=2.0, thr=0.5,
             mode="FLAPS 2s"),
        dict(name="level", dur=3.0, thr=0.45),
    ]),
    "kingfisher": dict(alt_ft=492, kts=12 / 0.51444, phases=[
        dict(name="level", dur=3.0, thr=0.5),
        dict(name="loop", done=rot_done(350), ele_ovr=-0.42, thr=1.0, track="pitch"),
        dict(name="level", dur=2.5, thr=0.5),
        dict(name="roll", done=rot_done(350), ail_ovr=0.6, thr=0.65, track="roll"),
        dict(name="level", dur=2.0, thr=0.5),
        dict(name="flaps-out", dur=3.0, flaps=1.0, pitch_t=6.0, thr=0.55,
             mode="FLAPS 2s"),
        dict(name="harrier", dur=6.0, flaps=1.0, pitch_t=22.0, thr=0.65,
             mode="FLAPS"),
        dict(name="flaps-in", dur=3.0, flaps=0.0, pitch_t=2.0, thr=0.55,
             mode="FLAPS 2s"),
        dict(name="level", dur=3.0, thr=0.5),
    ]),
    "dragonfly": dict(alt_ft=558, kts=16 / 0.51444, phases=[
        dict(name="level", dur=3.0, thr=0.85),
        dict(name="loop", done=rot_done(350), ele_ovr=-0.5, thr=1.0, track="pitch"),
        dict(name="level", dur=2.5, thr=0.85),
        dict(name="roll", done=rot_done(350), ail_ovr=0.8, thr=0.9, track="roll"),
        dict(name="level", dur=2.0, thr=0.85),
        dict(name="knife", dur=3.5, roll_t=90.0, rud=-0.85, thr=1.0,
             mode="KNIFE"),
        dict(name="level", dur=2.5, thr=0.85),
        # the honest one: full power + full pull cannot hold the nose up -
        # surfaces out of the prop stream, thrust line above the CG
        dict(name="hang-try", dur=7.0, pitch_t=85.0, thr=0.55, mode="NO HANG"),
        dict(name="recover", dur=3.0, pitch_t=0.0, thr=0.85),
        dict(name="level", dur=2.0, thr=0.85),
    ]),
}

if __name__ == "__main__":
    which = [a for a in sys.argv[1:] if a in MODELS] or list(MODELS)
    for model in which:
        cfg = MODELS[model]
        print(f"=== {model} ===", flush=True)
        fly(model, cfg["phases"], cfg["alt_ft"], cfg["kts"])
