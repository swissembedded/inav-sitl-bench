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
STEP = 10                      # downsample
rows = rows[::STEP]
t   = [float(r["t"]) for r in rows]
x   = [float(r["x"]) for r in rows]      # north
y   = [float(r["y"]) for r in rows]      # east
z   = [float(r["alt"]) for r in rows]
rpy = [(math.radians(float(r["js_roll"])), math.radians(float(r["js_pitch"])),
        math.radians(float(r["js_yaw"]))) for r in rows]
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
ias = [float(r["ias"]) for r in rows]
COL = {"level": "#1f77b4", "invert": "#d62728"}

def R_ned(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx

S = 60.0   # aircraft symbol scale [m]
SEGS = [((-0.6, 0, 0), (1.0, 0, 0)),          # fuselage
        ((0.1, -1.0, 0), (0.1, 1.0, 0)),      # wing
        ((-0.6, -0.35, 0), (-0.6, 0.35, 0)),  # tailplane
        ((-0.6, 0, 0), (-0.6, 0, -0.45))]     # fin (up = -z in NED)

fig = plt.figure(figsize=(11, 9))
# left column: 3D replay (top) + synchronized time strip (bottom).
# right column (x >= 0.72) is reserved for the info insets so nothing overlaps.
ax = fig.add_axes([0.01, 0.40, 0.66, 0.52], projection="3d")
ax2 = fig.add_axes([0.09, 0.07, 0.56, 0.24])
ax2.plot(t, [math.degrees(r[0]) for r in rpy], "#d62728", lw=1.4, label="roll")
ax2.plot(t, [math.degrees(r[1]) for r in rpy], "#1f77b4", lw=1.4, label="pitch")
ax2.axhline(180, color="gray", ls=":", lw=0.8); ax2.axhline(-180, color="gray", ls=":", lw=0.8)
PREP = ("settle", "cal", "armL", "armH", "level")
man_t = [tt for tt, p in zip(t, ph) if p == "manual"]
seq_t = [tt for tt, p in zip(t, ph) if p not in PREP and p != "manual"]
if man_t:
    ax2.axvspan(man_t[0], man_t[-1], color="0.5", alpha=0.10)
    ax2.text(0.5 * (man_t[0] + man_t[-1]), ax2.get_ylim()[1] * 0.9, "manual",
             ha="center", fontsize=8, color="0.35")
if seq_t:
    ax2.axvspan(seq_t[0], seq_t[-1], color="#d62728", alpha=0.08)
    ax2.axvline(seq_t[0], color="k", ls="--", lw=1.0)
    ax2.text(0.5 * (seq_t[0] + seq_t[-1]), ax2.get_ylim()[1] * 0.9, "sequence",
             ha="center", fontsize=8, color="#d62728")
ax2b = ax2.twinx()
ax2b.plot(t, ias, "#9467bd", lw=1.2, label="IAS")
ax2b.plot(t, z, "#2ca02c", lw=1.2, label="alt")
ax2b.set_ylabel("IAS [kts] / alt [m]")
ax2b.legend(fontsize=8, loc="upper right")
marker = ax2.axvline(t[0], color="k", lw=1.5)
ax2.set_xlabel("t [s]"); ax2.set_ylabel("deg"); ax2.legend(fontsize=8, loc="upper left"); ax2.grid(alpha=0.3)
ax.set_xlabel("east [m]"); ax.set_ylabel("north [m]"); ax.set_zlabel("alt [m]")
ax.set_xlim(min(y)-100, max(y)+100); ax.set_ylim(min(x)-100, max(x)+100)
ax.set_zlim(min(z)-50, max(z)+50)
trail, = ax.plot([], [], [], color="0.6", lw=1.2)
seg_lines = [ax.plot([], [], [], lw=2.5)[0] for _ in SEGS]
txt = ax.text2D(0.02, 0.95, "", transform=ax.transAxes, fontsize=10)
mtxt = ax.text2D(0.02, 0.88, "", transform=ax.transAxes, fontsize=14, fontweight="bold", color="#d62728")
# --- right column of insets (x >= 0.72), stacked top -> bottom ---
import os
if os.path.exists(f"jsbsim_params_{MAN}.txt"):
    ptext = "controller settings:" + chr(10) + open(f"jsbsim_params_{MAN}.txt").read()
    fig.text(0.72, 0.90, ptext, fontsize=7, family="monospace", va="top",
             bbox=dict(boxstyle="round", fc="0.95", ec="0.7"))
# pilot stick inputs (what the human commands)
axL = fig.add_axes([0.74, 0.50, 0.09, 0.10]); axR = fig.add_axes([0.87, 0.50, 0.09, 0.10])
for a_ in (axL, axR):
    a_.set_xlim(-1.2, 1.2); a_.set_ylim(-1.2, 1.2); a_.set_xticks([]); a_.set_yticks([])
    a_.axhline(0, color="0.85", lw=0.7); a_.axvline(0, color="0.85", lw=0.7)
axL.set_title("pilot thr/rud", fontsize=7); axR.set_title("pilot ail/ele", fontsize=7)
dotL, = axL.plot([0], [0], "o", ms=7, color="#1f77b4")
dotR, = axR.plot([0], [0], "o", ms=7, color="#1f77b4")
# control-surface commands the FC drives (aileron / elevator / rudder)
axS = fig.add_axes([0.74, 0.34, 0.22, 0.10])
axS.set_xlim(-1.1, 1.1); axS.set_ylim(-0.6, 2.6)
axS.set_yticks([0, 1, 2]); axS.set_yticklabels(["rudder", "elevator", "aileron"], fontsize=7)
axS.set_xticks([]); axS.axvline(0, color="0.85", lw=0.7)
axS.set_title("control surfaces (FC out)", fontsize=7)
bars = axS.barh([0, 1, 2], [0, 0, 0], height=0.6,
                color=["#2ca02c", "#1f77b4", "#d62728"])
ax.set_title(f"INAV orientation hold vs JSBSim -- {MAN}")
NOTES = {
    "inverted":    "Inverted flight: rolled 180 deg and held, controller keeps altitude at ~50 kts.",
    "knife_left":  "Knife-edge (left): rolls toward 90 deg, but the airframe has no fuselage lift, so it bleeds speed into a flat spin.",
    "knife_right": "Knife-edge (right): as knife_left -- without fuselage lift the speed decays and it mushes.",
    "hang":        "Prop-hang: nose held near vertical at near-zero airspeed; heading wanders.",
    "roll_hold":   "Axial roll with altitude assist: controller rolls while trying to hold height.",
    "floor_dive":  "Safety-floor deep dive: nose pushed down until the altitude floor catches and levels it.",
}
fig.text(0.5, 0.975, NOTES.get(MAN, ""), ha="center", va="top",
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
    txt.set_text(f"t={t[i]:5.1f}s  {ph[i].upper():7s}  roll={math.degrees(rpy[i][0]):+6.0f} deg  alt={z[i]:4.0f} m")
    marker.set_xdata([t[i], t[i]])
    mtxt.set_text(mode[i])
    dotL.set_data([st_rud[i]], [st_thr[i] * 2 - 1])
    dotR.set_data([st_ail[i]], [-st_ele[i]])
    for b, val in zip(bars, (cs_rud[i], cs_ele[i], cs_ail[i])):
        b.set_width(val)
    return seg_lines + [trail, txt, mtxt, marker, dotL, dotR] + list(bars)

anim = FuncAnimation(fig, frame, frames=len(rows), interval=60, blit=False)
outdir = "docs/videos"; os.makedirs(outdir, exist_ok=True)
outpath = f"{outdir}/jsbsim_{MAN}.mp4"
anim.save(outpath, writer=FFMpegWriter(fps=10, bitrate=1800), dpi=100)
print("written", outpath, len(rows), "frames")
