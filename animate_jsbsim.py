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
rows = list(csv.DictReader(open(f"jsbsim_log_{MAN}.csv")))
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

fig = plt.figure(figsize=(9, 9))
gs = fig.add_gridspec(3, 1, height_ratios=[2.4, 0.02, 1.0])
ax = fig.add_subplot(gs[0], projection="3d")
ax2 = fig.add_subplot(gs[2])
ax2.plot(t, [math.degrees(r[0]) for r in rpy], "#d62728", lw=1.4, label="roll")
ax2.plot(t, [math.degrees(r[1]) for r in rpy], "#1f77b4", lw=1.4, label="pitch")
ax2.axhline(180, color="gray", ls=":", lw=0.8); ax2.axhline(-180, color="gray", ls=":", lw=0.8)
inv = [tt for tt, p in zip(t, ph) if p == "invert"]
if inv: ax2.axvspan(inv[0], inv[-1], color="#d62728", alpha=0.08)
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
import os
if os.path.exists(f"jsbsim_params_{MAN}.txt"):
    ptext = "controller settings:
" + open(f"jsbsim_params_{MAN}.txt").read()
    fig.text(0.80, 0.62, ptext, fontsize=7, family="monospace", va="top",
             bbox=dict(boxstyle="round", fc="0.95", ec="0.7"))
ax.set_title(f"INAV orientation hold vs JSBSim -- {MAN}")

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
    return seg_lines + [trail, txt, mtxt, marker]

anim = FuncAnimation(fig, frame, frames=len(rows), interval=60, blit=False)
anim.save(f"jsbsim_{MAN}.mp4", writer=FFMpegWriter(fps=10, bitrate=1800), dpi=100)
print(f"written jsbsim_{MAN}.mp4,", len(rows), "frames")
