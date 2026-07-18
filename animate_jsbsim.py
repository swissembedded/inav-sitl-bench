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

# Renders jsbsim_log.csv into an animated 3D flight replay (GIF).
import csv, math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, FFMpegWriter
import imageio_ffmpeg
matplotlib.rcParams['animation.ffmpeg_path'] = imageio_ffmpeg.get_ffmpeg_exe()

import sys
MAN = sys.argv[1] if len(sys.argv) > 1 else "inverted"
_all = list(csv.DictReader(open(f"jsbsim_log_{MAN}.csv")))
_i0 = next((i for i, r in enumerate(_all) if r["phase"] in ("level", "manual")), 0)
rows = _all[_i0:]
# downsample to ~10 video frames per flight second, whatever the log rate
_ts = [float(r["t"]) for r in rows[:200]]
_dt = (_ts[-1] - _ts[0]) / max(1, len(_ts) - 1) if len(_ts) > 1 else 0.1
STEP = max(1, int(round(0.1 / _dt)))
rows = rows[::STEP]
t   = [float(r["t"]) for r in rows]
x   = [float(r["x"]) for r in rows]      # north
y   = [float(r["y"]) for r in rows]      # east
z   = [float(r["alt"]) for r in rows]
rpy = [(math.radians(float(r["js_roll"])), math.radians(float(r["js_pitch"])),
        math.radians(float(r["js_yaw"]))) for r in rows]
# GPS status (fix type, sats) -- older logs lack the columns
_FIX = {0: "no fix", 1: "GPS", 2: "GPS 2D", 3: "GPS 3D"}
gps = [(_FIX.get(int(r.get("gps_fix", 0) or 0), "?"), int(r.get("gps_sat", 0) or 0))
       for r in rows] if "gps_fix" in rows[0] else None
# truth (JSBSim) vs FC estimate, in degrees -- the validation pair:
# solid = physical truth (out of the plant), dashed = what the FC believes (in).
# Roll is UNWRAPPED: +180 and -180 are the same attitude, so a plane sitting
# inverted would otherwise paint +179/-179 sawtooth "fireworks" instead of a
# calm line at 180.
def unwrap_deg(vals):
    return list(np.degrees(np.unwrap(np.radians(vals))))
js_roll  = unwrap_deg([float(r["js_roll"])  for r in rows])
fc_roll  = unwrap_deg([float(r["fc_roll"])  for r in rows])
# align the FC branch (it may unwrap to 180 vs -180 = same attitude)
_off = 360.0 * round((np.mean(js_roll) - np.mean(fc_roll)) / 360.0)
fc_roll = [v + _off for v in fc_roll]
js_pitch = [float(r["js_pitch"]) for r in rows]
fc_pitch = [float(r["fc_pitch"]) for r in rows]
ph  = [r["phase"] for r in rows]
mode = [r.get("mode", "") for r in rows]
def stn(k, mid=1500.0, rng=500.0):
    return [ (float(r.get(k, 1500)) - mid) / rng for r in rows ]
st_ail, st_ele = stn("st_ail"), stn("st_ele")
st_thr = [ (float(r.get("st_thr", 1000)) - 1000.0) / 1000.0 for r in rows ]
st_rud = stn("st_rud")
# FC control-surface commands (normalized -1..1), i.e. what the controller
# drives onto aileron / elevator / rudder while the pilot sticks stay put.
cs_ail = [float(r.get("ail", 0)) for r in rows]
cs_ele = [float(r.get("ele", 0)) for r in rows]
cs_rud = [float(r.get("rud", 0)) for r in rows]
sw = {k: [float(r.get(k, 1000)) for r in rows] for k in ("st_arm","st_angle","st_inv","st_sel")}
cs_thr = [float(r.get("thr", 0)) for r in rows]        # 0..1 throttle driven into the plant
fc_thr = [float(r.get("fc_thr", r.get("thr", 0))) for r in rows]  # FC's own throttle output
tvc_p = [float(r.get("tvc_p", 0)) for r in rows]   # vectored nozzle servos
tvc_y = [float(r.get("tvc_y", 0)) for r in rows]
HAS_TVC = MAN == "hang_tvc"
flap = [float(r.get("flap", 0)) for r in rows]   # 0..1 deployed
HAS_FLAP = True   # identical panel in every video
ias = [float(r["ias"]) for r in rows]
fc_alt = [float(r.get("fc_alt", r["alt"])) for r in rows]  # FC baro-estimated altitude
# baro is referenced to the boot zero (AGL), truth is MSL -- shift the baro
# trace by the constant start offset so the two overlay: a faithful baro then
# sits on the truth line and any drift shows up as a gap
_baro_off = (z[0] - fc_alt[0]) if fc_alt else 0.0
fc_alt = [v + _baro_off for v in fc_alt]
COL = {"level": "#1f77b4", "invert": "#d62728"}

def R_ned(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx

SEGS = [((-0.6, 0, 0), (1.0, 0, 0)),          # fuselage
        ((0.1, -1.0, 0), (0.1, 1.0, 0)),      # wing
        ((-0.6, -0.35, 0), (-0.6, 0.35, 0)),  # tailplane
        ((-0.6, 0, 0), (-0.6, 0, -0.45))]     # fin (up = -z in NED)

fig = plt.figure(figsize=(11, 9))
# left column: 3D replay (top) + TWO synchronized time graphs (bottom):
# roll (truth vs FC estimate, unwrapped) and pitch (truth vs FC estimate)
# with altitude/IAS on the second axis. solid = JSBSim truth, dashed = FC
# estimate; if the loop is valid they overlap.
# right column (x >= 0.72) is reserved for the info insets so nothing overlaps.
ax = fig.add_axes([0.01, 0.47, 0.66, 0.47], projection="3d")
axRoll = fig.add_axes([0.09, 0.325, 0.56, 0.125])
axPit  = fig.add_axes([0.09, 0.175, 0.56, 0.125], sharex=axRoll)
axOut  = fig.add_axes([0.09, 0.045, 0.56, 0.10], sharex=axRoll)
axRoll.plot(t, js_roll, "#d62728", lw=1.4, label="roll truth")
axRoll.plot(t, fc_roll, "k", lw=1.0, ls="--", label="roll FC est")
axRoll.axhline(180, color="gray", ls=":", lw=0.8)
axRoll.axhline(-180, color="gray", ls=":", lw=0.8)
axRoll.axhline(90, color="gray", ls=":", lw=0.5)
axRoll.axhline(-90, color="gray", ls=":", lw=0.5)
axRoll.set_ylabel("roll [deg]")
axRoll.legend(fontsize=7, loc="upper left")
axRoll.grid(alpha=0.3)
plt.setp(axRoll.get_xticklabels(), visible=False)
axPit.plot(t, js_pitch, "#1f77b4", lw=1.4, label="pitch truth")
axPit.plot(t, fc_pitch, "k", lw=1.0, ls="--", label="pitch FC est")
axPit.set_ylabel("pitch [deg]")
plt.setp(axPit.get_xticklabels(), visible=False)
axPit.legend(fontsize=7, loc="upper left")
axPit.grid(alpha=0.3)
axPb = axPit.twinx()
axPb.plot(t, ias, "#9467bd", lw=1.0, label="IAS")
axPb.plot(t, z, "#2ca02c", lw=1.4, label="alt truth")
axPb.plot(t, fc_alt, "#2ca02c", lw=1.0, ls="--", label="alt meas (baro)")
axPb.plot(t, [v * 100 for v in fc_thr], "#ff7f0e", lw=1.0, label="thrust [%]")
axPb.set_ylabel("IAS [kts] / alt [m] / thr [%]", fontsize=8)
# the safety floor is the whole story of the floor maneuvers - draw it
# (alt_floor_altitude = 30 m over the arming baro zero ~ first logged alt)
FLOOR_ABS = z[0] + 30 if MAN in ("floor_dive", "floor_spin", "floor_panic") else None
if FLOOR_ABS is not None:
    axPb.axhline(FLOOR_ABS, color="#d62728", lw=1.3, ls=":")
    axPb.axhline(FLOOR_ABS + 10, color="#d62728", lw=0.7, ls=":", alpha=0.4)
    axPb.text(t[0], FLOOR_ABS + 1, " FLOOR", color="#d62728", fontsize=7, va="bottom")
axPb.legend(fontsize=7, loc="upper right")
PREP = ("settle", "cal", "armL", "armH", "level")
gust_t = [tt for tt, p in zip(t, ph) if p == "gust"]
man_t = [tt for tt, p in zip(t, ph) if p == "manual"]
seq_t = [tt for tt, p in zip(t, ph) if p not in PREP and p != "manual"]
for a_ in (axRoll, axPit, axOut):
    if man_t:
        a_.axvspan(man_t[0], man_t[-1], color="0.5", alpha=0.10)
    if seq_t:
        a_.axvspan(seq_t[0], seq_t[-1], color="#d62728", alpha=0.08)
        a_.axvline(seq_t[0], color="k", ls="--", lw=1.0)
if man_t:
    axRoll.text(0.5 * (man_t[0] + man_t[-1]), axRoll.get_ylim()[1] * 0.9, "manual",
                ha="center", fontsize=8, color="0.35")
if gust_t:
    for a_ in (axRoll, axPit, axOut):
        a_.axvspan(gust_t[0], gust_t[-1], color="#17becf", alpha=0.18, zorder=0)
    axRoll.text(0.5 * (gust_t[0] + gust_t[-1]), axRoll.get_ylim()[0] * 0.85, "gust",
                ha="center", fontsize=8, color="#0e7a8a")
if seq_t:
    axRoll.text(0.5 * (seq_t[0] + seq_t[-1]), axRoll.get_ylim()[1] * 0.9, "sequence",
                ha="center", fontsize=8, color="#d62728")
# FC outputs over time: a flat zero line here means the controller is NOT
# correcting -- this is the trace that exposes a dead assist immediately
axOut.plot(t, cs_ele, "#1f77b4", lw=1.2, label="elevator")
axOut.plot(t, cs_ail, "#d62728", lw=0.9, label="aileron")
axOut.plot(t, cs_rud, "#2ca02c", lw=0.9, label="rudder")
axOut.plot(t, fc_thr, "#ff7f0e", lw=0.9, label="throttle")
if HAS_TVC:
    axOut.plot(t, tvc_p, "#9467bd", lw=1.2, label="tvc pitch")
    axOut.plot(t, tvc_y, "#8c564b", lw=0.9, label="tvc yaw")
axOut.set_ylim(-1.05, 1.05)
axOut.set_ylabel("FC out", fontsize=8)
axOut.set_xlabel("t [s]")
axOut.legend(fontsize=6.5, loc="upper left", ncol=4)
axOut.grid(alpha=0.3)
marker = axRoll.axvline(t[0], color="k", lw=1.5)
markerP = axPit.axvline(t[0], color="k", lw=1.5)
markerO = axOut.axvline(t[0], color="k", lw=1.5)
ax.set_xlabel("east [m]"); ax.set_ylabel("north [m]"); ax.set_zlabel("alt [m]")
# fixed isotropic cube around the WHOLE track: the full flight path stays
# visible, the box has the same shape in every video, and nothing is
# squashed. The aircraft symbol scales with the track so it stays visible.
_cx, _cy, _cz = (min(y)+max(y))/2, (min(x)+max(x))/2, (min(z)+max(z))/2
L = max(max(y)-min(y), max(x)-min(x), max(z)-min(z)) / 2 + 60
_z0 = max(0.0, _cz - L)   # never show below ground
ax.set_xlim(_cx-L, _cx+L); ax.set_ylim(_cy-L, _cy+L); ax.set_zlim(_z0, _z0 + 2*L)
ax.set_box_aspect((1, 1, 1))
S = max(25.0, L / 5.0)   # aircraft symbol scale, relative to the scene
if FLOOR_ABS is not None:
    # translucent floor plane in the 3D view: the spin/dive visibly falls
    # onto it and the catch reads as a catch
    import numpy as _np
    _gx, _gy = _np.meshgrid([_cx-L, _cx+L], [_cy-L, _cy+L])
    ax.plot_surface(_gx, _gy, _np.full_like(_gx, FLOOR_ABS),
                    color="#d62728", alpha=0.12, zorder=0)
trail, = ax.plot([], [], [], color="0.6", lw=1.2)
seg_lines = [ax.plot([], [], [], lw=2.5)[0] for _ in SEGS]
txt = ax.text2D(0.02, 0.95, "", transform=ax.transAxes, fontsize=10)
mtxt = ax.text2D(0.02, 0.88, "", transform=ax.transAxes, fontsize=14, fontweight="bold", color="#d62728")

# EVENT NARRATION in the film itself (Daniel): a per-frame caption derived
# from the LOGGED data - phase labels for the figures, and the FW safety
# word overrides everything the moment a recovery owns the aircraft.
PHASE_TEXT = {
    "einflug": "trim: the controller finds its level throttle",
    "base": "level base line", "base-spin": "level base line",
    "base-floor": "level base line",
    "inverted": "INVERTED HOLD",
    "inv-bleed": "bleeding speed inside the hold before release",
    "roll": "AXIAL ROLL", "loop": "LOOP",
    "knife-L": "KNIFE EDGE LEFT", "knife-R": "KNIFE EDGE RIGHT",
    "knife_left": "KNIFE EDGE LEFT", "knife_right": "KNIFE EDGE RIGHT",
    "spin-hold": "FLAT SPIN: attitude held flat",
    "spin-rud": "FLAT SPIN: the pilot's rudder drives the rotation",
    "rud-release": "rudder released: rotation stops, attitude still held",
    "flaps-out": "flaps deploying (slow 2 s servo)",
    "harrier": "BLOWN-FLAP HARRIER: high alpha on the prop wash",
    "flaps-slow": "FLAPS SLOW PASS",
    "hang": "PROP HANG: the controller finds its hover throttle",
    "bleed": "bleeding excess speed on a shallow downline",
    "exit": "exit to level",
    "floor-dive": "HELD DIVE into the floor line...",
    "dive-held": "HELD DIVE into the floor line...",
    "dive-chop": "PANIC DIVE: throttle chopped, elevator held...",
    "dive-nofloor": "same dive, floor OFF: it punches through",
    "cruise": "cruise",
    "slow-decay": "throttle cut: the rotor is starving",
    "tip-window": "rotor starved: tilt authority is gone",
    "after": "throttle returned",
    "land-idle": "throttle to IDLE: landing - the guard stands down by design",
    "gust": "downdraft gust",
    "takeover": "pilot takes over: fresh stick input releases the floor",
}


def annotate(i):
    s = int(float(rows[i].get("safety", 0) or 0))
    if s & 4:
        return ("CAUGHT: rotor guard - thrust restores the rotor", "#d62728")
    if s & 2:
        return ("CAUGHT: the floor flies the recovery", "#d62728")
    ph_i = ph[i]
    alt_i = z[i]
    roll_i = abs(math.degrees(rpy[i][0]))
    if ph_i in ("slow-decay", "tip-window", "after", "land-idle") and alt_i < 1.5:
        return ("IMPACT", "#d62728")
    if ph_i in ("slow-decay", "tip-window", "land-idle") and roll_i > 60:
        return ("TIPPING OVER", "#d62728")
    return (PHASE_TEXT.get(ph_i, ""), "0.25")


anntxt = ax.text2D(0.5, 0.99, "", transform=ax.transAxes, fontsize=12,
                   fontweight="bold", ha="center", va="top")
# --- right column: four identical panels (same x, width, height, spacing) ---
import os
PX, PW, PH = 0.74, 0.22, 0.155
PY = [0.775, 0.565, 0.355, 0.145]
def panel(y, title):
    p = fig.add_axes([PX, y, PW, PH])
    p.set_title(title, fontsize=8.5, fontweight="bold", loc="left")
    return p

# 1) controller settings (static text)
axSet = panel(PY[0], "controller settings")
axSet.set_xticks([]); axSet.set_yticks([])
if os.path.exists(f"jsbsim_params_{MAN}.txt"):
    axSet.text(0.04, 0.97, open(f"jsbsim_params_{MAN}.txt").read(), fontsize=6.8,
               family="monospace", va="top", transform=axSet.transAxes)

# 2) controller IN: pilot sticks (two crosses in one panel)
axIN = panel(PY[1], "controller IN: pilot sticks")
axIN.set_xlim(-2.7, 2.7); axIN.set_ylim(-1.55, 1.55)
axIN.set_xticks([]); axIN.set_yticks([])
for cx, lbl in ((-1.4, "thr/rud"), (1.4, "ail/ele")):
    axIN.plot([cx-1, cx+1], [0, 0], color="0.85", lw=0.8)
    axIN.plot([cx, cx], [-1, 1], color="0.85", lw=0.8)
    axIN.text(cx, 1.18, lbl, ha="center", fontsize=7, color="0.35")
dotL, = axIN.plot([-1.4], [0], "o", ms=7, color="#1f77b4")
dotR, = axIN.plot([1.4], [0], "o", ms=7, color="#1f77b4")

# model identity (from the params file): drives the actuator-true OUT
# panel below AND the switch labels - read it before the panels are drawn
MODEL = None
if os.path.exists(f"jsbsim_params_{MAN}.txt"):
    for _ln in open(f"jsbsim_params_{MAN}.txt"):
        if _ln.startswith("model="):
            MODEL = _ln.strip().split("=", 1)[1]
_ACT = None
if MODEL:
    try:
        from airframe_config import AIRFRAMES as _AF
        _ACT = _AF[MODEL][0]
    except Exception:
        _ACT = None

# 3) pilot mode switches (lever position = channel value)
axSw = panel(PY[2], "switches")
# DETENT logic, identical in EVERY video: one detent per provisioned band
# plus an explicit OFF; the lever snaps onto the active band's label
# (labels = the FC box names). Bands mirror bench.py provisioning.
SW_BANDS = {
    "st_angle": [("OFF", 0), ("F LOOP", 1150), ("FLAT SPIN", 1300),
                 ("F ROLL", 1450), ("F SEQ", 1600), ("ANGLE", 1750)],
    "st_inv":   [("OFF", 0), ("FLOOR", 1700)],
    "st_sel":   [("OFF", 0), ("INVERT", 1150), ("KNIFE L", 1390),
                 ("KNIFE R", 1630), ("P-HANG", 1870)],
}
if _ACT == "GYRO":
    # the gyro provisions ROTOR GUARD on the SELECT channel instead of
    # the attitude presets - label the lever with what the FC really has
    SW_BANDS["st_sel"] = [("OFF", 0), ("ROTOR GUARD", 1700)]
SW_COLS = ["st_angle", "st_inv", "st_sel"]

def sw_detent(key, val):
    idx = 0
    for i, (_, lo) in enumerate(SW_BANDS[key]):
        if val >= lo:
            idx = i
    return idx

def sw_y(key, idx):
    return -1.0 + 2.0 * idx / (len(SW_BANDS[key]) - 1)

axSw.set_xlim(-0.5, 3.4); axSw.set_ylim(-1.35, 1.35)
axSw.set_xticks([0, 1, 2])
axSw.set_xticklabels(["MODE", "FLOOR", "SEL"], fontsize=7.5)
axSw.set_yticks([])
for xx, key in enumerate(SW_COLS):
    axSw.plot([xx, xx], [-1, 1], color="0.8", lw=3, solid_capstyle="round")
    for i, (lbl, _) in enumerate(SW_BANDS[key]):
        axSw.text(xx + 0.14, sw_y(key, i), lbl, fontsize=6.0,
                  va="center", color="0.35")
levers, = axSw.plot([0, 1, 2], [-1, -1, -1], "s", ms=8, color="#d62728")

# 4) controller OUT: FC commands (instant bars). ACTUATOR-TRUE: the bars
# show exactly the surfaces the airframe HAS (PANEL_BARS per actuator set
# in airframe_config, model name from the params file); the elevon bars
# are the FC's own roll+pitch mix (the rate-50 smix rules). Logs without
# a model line keep the legacy fixed panel.
_out_labels = None
if _ACT:
    try:
        from airframe_config import PANEL_BARS as _PB
        _out_labels = list(_PB[_ACT])
    except Exception:
        _out_labels = None
if _out_labels is None:
    _out_labels = ["throttle", "rudder", "elevator", "aileron"]
    if HAS_TVC:
        _out_labels += ["tvc yaw", "tvc pitch"]
    if HAS_FLAP:
        _out_labels += ["flaps"]
_OUT_SRC = {
    "throttle": fc_thr, "rudder": cs_rud, "elevator": cs_ele,
    "aileron": cs_ail, "flaps": flap, "tvc yaw": tvc_y, "tvc pitch": tvc_p,
    "elevon L": [0.5 * a + 0.5 * e for a, e in zip(cs_ail, cs_ele)],
    "elevon R": [-0.5 * a + 0.5 * e for a, e in zip(cs_ail, cs_ele)],
    "rotor tilt": cs_ail, "pre-rotator": [0.0] * len(rows),
}
_OUT_COL = {"throttle": "#ff7f0e", "rudder": "#2ca02c", "elevator": "#1f77b4",
            "aileron": "#d62728", "flaps": "#7f7f7f", "tvc yaw": "#8c564b",
            "tvc pitch": "#9467bd", "elevon L": "#d62728", "elevon R": "#1f77b4",
            "rotor tilt": "#d62728", "pre-rotator": "#7f7f7f"}
_out_series = [_OUT_SRC[b] for b in _out_labels]
_out_colors = [_OUT_COL.get(b, "0.4") for b in _out_labels]
axS = panel(PY[3], "controller OUT: FC commands")
_n_out = len(_out_labels)
axS.set_xlim(-1.1, 1.1); axS.set_ylim(-0.6, _n_out - 0.4)
axS.set_yticks([])
for yy, lbl in enumerate(_out_labels):
    axS.text(-1.05, yy + 0.38, lbl, fontsize=6.5, va="center", color="0.35")
axS.set_xticks([-1, 0, 1]); axS.set_xticklabels(["-1", "0", "1"], fontsize=6)
axS.axvline(0, color="0.85", lw=0.7)
bars = axS.barh(list(range(_n_out)), [0] * _n_out, height=0.6, color=_out_colors)
NOTES = {
    "inverted":    "Inverted flight: target slews to 180 deg at the entry rate, altitude assist holds height through a gust and a deliberate rudder turn.",
    "knife_left":  "Knife-edge (left): held at -90 deg with nose-up pitch trim carrying the fuselage lift on the rudder; altitude assist keeps the height.",
    "knife_right": "Knife-edge (right): as knife_left on the other side -- separate trim per side, prop effects break the symmetry.",
    "hang":        "Prop-hang: nose held near vertical, hover throttle PID owns the altitude (the pull converts speed to height first); heading is the free axis.",
    "roll_hold":   "Axial roll with altitude assist: earth-referenced nose-up distributes to elevator and rudder as the roll phase demands.",
    "floor_dive":  "Safety floor: held dive is caught at the floor; then same dive with the floor switched OFF punches through.",
    "hang_tvc":    "Prop hang on a pusher delta: elevons are dead at zero airspeed, ALL control authority comes from the vectored nozzle (thrust vectoring with inverse throttle compensation).",
    "flat_spin":   "FLAT SPIN flight mode: the controller holds roll and pitch FLAT while the pilot's full rudder at idle drives the autorotation; releasing the rudder stops the rotation with the attitude still held, releasing the box recovers.",
    "inv_spin":    "INVERTED FLAT SPIN: FLAT SPIN box + INVERTED selects the held attitude; the rudder commands rotation about the earth vertical with aircraft-referenced stick sense (seen from above the rotation reverses vs upright, exactly like a real aircraft).",
    "knife_spin":  "KNIFE EDGE SPIN: FLAT SPIN box + KNIFE L holds the edge while the rudder command distributes onto the body pitch axis - the rotation about the vertical that the knife attitude leaves free.",
    "inverted_stick": "Stick carving around the inverted reference: half aileron is a HELD angle offset (not a rate), releasing returns the target gently; then the same on the elevator, where the pilot owns the altitude and the assist yields.",
    "loop_fig":    "Full loop at fig_loop_rate under full power, closing on the entry altitude; the level hold with assist settles afterwards.",
    "floor_spin":  "Safety floor vs FLAT SPIN: the autorotation trundles down at idle; the floor recovery overrides the spin, rolls upright out of the rotation and climbs out on its own throttle floor.",
    "floor_panic": "Safety floor, panic case: throttle chopped and down-elevator held through the dive - the catch suppresses the held stick and brings its own throttle floor for the climb.",
}
# --title "<text>" overrides the built-in note (sequence videos carry the
# name of the routine that was programmed, not a fixed maneuver blurb)
TITLE = sys.argv[sys.argv.index("--title") + 1] if "--title" in sys.argv else None
fig.text(0.5, 0.975, TITLE or NOTES.get(MAN, ""), ha="center", va="top",
         fontsize=9, style="italic", wrap=True)

def frame(i):
    R = R_ned(*rpy[i])
    c = COL.get(ph[i], "0.4")
    for ln, (a, b) in zip(seg_lines, SEGS):
        pa, pb = R @ np.array(a) * S, R @ np.array(b) * S
        # NED body->world: world x=north y=east z=down; plot: (east, north, up)
        ln.set_data([y[i]+pa[1], y[i]+pb[1]], [x[i]+pa[0], x[i]+pb[0]])
        ln.set_3d_properties([z[i]-pa[2], z[i]-pb[2]])
        ln.set_color(c)
    trail.set_data(y[:i+1], x[:i+1]); trail.set_3d_properties(z[:i+1])
    gtxt = ""
    if gps is not None:
        gtxt = f"  {gps[i][0]}" + (f" {gps[i][1]}sat" if gps[i][1] else "")
    _arm = "ARMED" if sw["st_arm"][i] > 1500 else "DISARMED"
    txt.set_text(f"t={t[i]:5.1f}s  {_arm}  {ph[i].upper():7s}  "
                 f"roll={math.degrees(rpy[i][0]):+6.0f}  pitch={math.degrees(rpy[i][1]):+5.0f}  "
                 f"yaw={math.degrees(rpy[i][2]) % 360.0:3.0f} deg  alt={z[i]:4.0f} m{gtxt}")
    marker.set_xdata([t[i], t[i]])
    markerP.set_xdata([t[i], t[i]])
    markerO.set_xdata([t[i], t[i]])
    mtxt.set_text(mode[i])
    _a, _c = annotate(i)
    anntxt.set_text(_a)
    anntxt.set_color(_c)
    dotL.set_data([-1.4 + st_rud[i]], [st_thr[i] * 2 - 1])
    dotR.set_data([1.4 + st_ail[i]], [-st_ele[i]])
    for b, series in zip(bars, _out_series):
        b.set_width(series[i])
    levers.set_data([0, 1, 2],
                    [sw_y(k, sw_detent(k, sw[k][i])) for k in SW_COLS])
    return seg_lines + [trail, txt, mtxt, anntxt, marker, markerP, markerO, dotL, dotR, levers] + list(bars)

anim = FuncAnimation(fig, frame, frames=len(rows), interval=60, blit=False)
outdir = "docs/videos"; os.makedirs(outdir, exist_ok=True)
outpath = f"{outdir}/jsbsim_{MAN}.mp4"
# AV1 (libaom), quality-based: ~5x smaller than the old fixed-bitrate
# H.264 for this line-graphics content -- the videos live in the repo and
# every regeneration adds full new blobs to the history. Trade-off:
# Safari plays AV1 only on the newest hardware; Chrome/Edge/Firefox
# decode it everywhere (dav1d).
anim.save(outpath, writer=FFMpegWriter(fps=10, codec="libaom-av1",
                                       extra_args=["-crf", "34", "-b:v", "0",
                                                   "-cpu-used", "6", "-row-mt", "1",
                                                   "-pix_fmt", "yuv420p",
                                                   "-movflags", "+faststart"]), dpi=100)
print("written", outpath, len(rows), "frames")
