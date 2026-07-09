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

# Closed loop: INAV Windows-SITL <-> JSBSim via the proven MSP_SIMULATOR path.
# Phases: settle AHRS -> arm -> ANGLE level (sign/convention check) -> INVERT.
# Usage: python jsbsim_fly.py [--flip-ele] [--flip-ail] [--flip-rud]
import sys, struct, time, math

from msp import MspClient
from hitl import sim_step
from jsbsim_plant import JSBSimPlant

FLIP_ELE = "--flip-ele" in sys.argv
FLIP_AIL = "--flip-ail" in sys.argv
FLIP_RUD = "--flip-rud" in sys.argv

RC_LOW, RC_MID, RC_HIGH = 1000, 1500, 2000
# channels: A E T R ARM ANGLE INVERT SELECT  (bench provisioning layout)
def rc_ch(thr=RC_LOW, arm=RC_LOW, angle=RC_LOW, invert=RC_LOW, sel=RC_LOW, ele=RC_MID):
    return [RC_MID, ele, thr, RC_MID, arm, angle, invert, sel]

def mode_of(rc):
    if rc[5] > 1700: base = "ANGLE"
    elif rc[6] > 1700: base = "INVERT"
    elif 1450 < rc[6] < 1700: base = "F ROLL"
    elif 1150 < rc[6] < 1450: base = "FLOOR"
    elif 1150 < rc[7] < 1450: base = "KNIFE L"
    elif 1450 < rc[7] < 1750: base = "KNIFE R"
    elif rc[7] >= 1750: base = "P-HANG"
    else: base = "ACRO"
    if base != "FLOOR" and 1150 < rc[6] < 1450: base += "+FLOOR"
    return base

FLAG_ARMED = 1 << 2
FLAG_CAL = 1 << 9

def arming_flags(m):
    p = m.request(0x2000)
    return struct.unpack("<I", p[9:13])[0]

def fc_att(m):
    p = m.request(108)
    r, pi, y = struct.unpack("<hhh", p[:6])
    return r / 10.0, pi / 10.0, y

m = MspClient()
plant = JSBSimPlant()
log = open(r"jsbsim_log.csv", "w")
log.write("t,phase,fc_roll,fc_pitch,fc_yaw,js_roll,js_pitch,js_yaw,ias,alt,ail,ele,thr,x,y\n")
T0 = time.time()

def loop(secs, phase, rc, thr_override=None, print_every=1.0):
    last = 0.0
    t0 = time.time()
    it_prev = time.perf_counter()
    while time.time() - t0 < secs:
        it0 = time.perf_counter()
        r = sim_step(m, plant.acc_mg(), plant.gyro_dps16(), rc, baro_pa=plant.baro_pa())
        ail = -r.stab_roll if FLIP_AIL else r.stab_roll
        ele = -r.stab_pitch if FLIP_ELE else r.stab_pitch
        rud = -r.stab_yaw if FLIP_RUD else r.stab_yaw
        thr = thr_override if thr_override is not None else (r.stab_throttle + 1.0) / 2.0
        plant.set_controls(ail, ele, rud, thr)
        now = time.perf_counter()
        plant.step(dt=now - it_prev)      # sim time == real time, adaptively
        it_prev = now
        jr, jp, jy = plant.rpy()
        fr, fp, fy = r.att_roll_deg, r.att_pitch_deg, r.att_yaw_deg   # aus der Reply -- keine Extra-Roundtrips
        t = time.time() - T0
        log.write(f"{t:.2f},{phase},{fr:.1f},{fp:.1f},{fy:.0f},"
                  f"{jr:.1f},{jp:.1f},{jy:.1f},{plant.ias_kts():.0f},{plant.z:.1f},{ail:.2f},{ele:.2f},{thr:.2f},"
                  f"{plant.xy()[0]:.1f},{plant.xy()[1]:.1f}\n")
        if time.time() - last > print_every:
            print(f"  [{phase:7}] FC {fr:+7.1f}/{fp:+6.1f}/{fy:3.0f} | JS {jr:+7.1f}/{jp:+6.1f}/{jy:5.1f} | "
                  f"IAS {plant.ias_kts():3.0f} alt {plant.z:5.0f} ele {ele:+.2f}")
            last = time.time()
        s = 0.01 - (time.perf_counter() - it0)   # cap at ~100 Hz
        if s > 0:
            time.sleep(s)

print("boot-cal abwarten (bench-Routine)...")
from bench import wait_boot_calibration
wait_boot_calibration(m)

print("=== SETTLE (AHRS an JSBSim angleichen, disarmed) ===")
loop(6, "settle", rc_ch())
fr, fp, fy = fc_att(m); jr, jp, jy = plant.rpy()
print(f"Konventions-Check: FC {fr:+.1f}/{fp:+.1f} vs JS {jr:+.1f}/{jp:+.1f}  "
      f"({'OK' if abs(fr-jr)<15 and abs(fp-jp)<15 else 'MISMATCH -> Vorzeichen pruefen'})")

print("=== CAL (HITL-Stream laufen lassen bis bit9 weg) ===")
t0 = time.time()
while (arming_flags(m) & FLAG_CAL) and time.time() - t0 < 25:
    loop(1.0, "cal", rc_ch(angle=RC_HIGH), print_every=4)
print("cal fertig, flags=0x%X" % arming_flags(m))

print("=== ARM (Toggle-Zyklen bis ARMED) ===")
t0 = time.time()
while not (arming_flags(m) & FLAG_ARMED) and time.time() - t0 < 20:
    loop(1.0, "armL", rc_ch(thr=RC_LOW, arm=RC_LOW, angle=RC_HIGH), print_every=9)
    loop(1.2, "armH", rc_ch(thr=RC_LOW, arm=RC_HIGH, angle=RC_HIGH), print_every=9)
print("ARMED:", bool(arming_flags(m) & FLAG_ARMED), f"flags=0x{arming_flags(m):X}")

print("=== ANGLE LEVEL (Vorzeichen-Beweis: muss level bleiben) ===")
loop(8, "level", rc_ch(thr=1700, arm=RC_HIGH, angle=RC_HIGH))

MAN = next((a for a in sys.argv[1:] if not a.startswith("--")), "inverted")
MAN_RC = {
    "inverted":    dict(invert=RC_HIGH),
    "knife_left":  dict(sel=1300),
    "knife_right": dict(sel=1600),
    "hang":        dict(sel=1900),
    "roll_hold":   dict(invert=1575),
    "floor_dive":  dict(angle=RC_HIGH, invert=1300),
}[MAN]
thrM = 1500 if MAN == "hang" else 1700   # hang: Stick mitte -> hover throttle owns
print(f"=== MANOEUVRE {MAN} ===")
if MAN == "floor_dive":
    loop(4, "arm-floor", rc_ch(thr=1700, arm=RC_HIGH, **MAN_RC), print_every=1)
    loop(7, "dive", rc_ch(thr=1700, arm=RC_HIGH, ele=1100, **MAN_RC), print_every=0.7)
    loop(12, "recover", rc_ch(thr=1700, arm=RC_HIGH, **MAN_RC), print_every=0.7)
else:
    loop(16, "invert", rc_ch(thr=thrM, arm=RC_HIGH, angle=RC_LOW, **MAN_RC), print_every=0.7)

fr, fp, fy = fc_att(m); jr, jp, jy = plant.rpy()
print(f"FINAL: FC roll {fr:+.1f}  JS roll {jr:+.1f}  (Erfolg wenn |roll| ~ 180)")
log.close(); m.close()
