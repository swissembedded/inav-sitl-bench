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

# Post-flight visualisation of jsbsim_log.csv -> jsbsim_flight.png
import csv
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

rows = list(csv.DictReader(open(r"jsbsim_log.csv")))
t = [float(r["t"]) for r in rows]
g = lambda k: [float(r[k]) for r in rows]
phase = [r["phase"] for r in rows]
PH_COL = {"settle": "0.7", "cal": "0.6", "armL": "0.5", "armH": "0.5",
          "level": "#1f77b4", "invert": "#d62728"}

fig = plt.figure(figsize=(13, 8))
ax1 = fig.add_subplot(2, 2, 1)
ax1.plot(t, g("fc_roll"), "#d62728", lw=1.6, label="FC roll")
ax1.plot(t, g("js_roll"), "#d62728", lw=0.8, ls="--", alpha=0.6, label="JSBSim roll")
ax1.plot(t, g("fc_pitch"), "#1f77b4", lw=1.6, label="FC pitch")
ax1.plot(t, g("js_pitch"), "#1f77b4", lw=0.8, ls="--", alpha=0.6, label="JSBSim pitch")
ax1.axhline(180, color="gray", ls=":"); ax1.axhline(-180, color="gray", ls=":")
for a, b, ph in [(t[i], t[i+1], phase[i]) for i in range(len(t)-1) if phase[i] == "invert"]:
    ax1.axvspan(a, b, color="#d62728", alpha=0.04)
ax1.set_title("Attitude: FC estimate vs JSBSim truth (INVERT shaded)")
ax1.set_xlabel("t [s]"); ax1.set_ylabel("deg"); ax1.legend(fontsize=8); ax1.grid(alpha=0.3)

ax2 = fig.add_subplot(2, 2, 2)
ax2.plot(t, g("alt"), "#2ca02c", lw=1.6, label="altitude [m]")
ax2b = ax2.twinx()
ax2b.plot(t, g("ias"), "#9467bd", lw=1.2, label="IAS [kts]")
ax2.set_title("Altitude / airspeed"); ax2.set_xlabel("t [s]")
ax2.set_ylabel("alt [m]", color="#2ca02c"); ax2b.set_ylabel("IAS [kts]", color="#9467bd")
ax2.grid(alpha=0.3)

ax3 = fig.add_subplot(2, 2, 3)
ax3.plot(t, g("ele"), lw=1.0, label="elevator")
ax3.plot(t, g("ail"), lw=1.0, label="aileron")
ax3.plot(t, g("thr"), lw=1.0, label="throttle")
ax3.set_title("Controller outputs"); ax3.set_xlabel("t [s]"); ax3.legend(fontsize=8); ax3.grid(alpha=0.3)

ax4 = fig.add_subplot(2, 2, 4, projection="3d")
x, y, z = g("x"), g("y"), g("alt")
for i in range(0, len(t) - 1, 2):
    ax4.plot(y[i:i+3], x[i:i+3], z[i:i+3], color=PH_COL.get(phase[i], "k"), lw=1.2)
ax4.set_title("Flight path (blue=ANGLE, red=INVERT)")
ax4.set_xlabel("east [m]"); ax4.set_ylabel("north [m]"); ax4.set_zlabel("alt [m]")

fig.suptitle("INAV quaternion orientation hold vs JSBSim full aerodynamics (closed loop, headless)")
fig.tight_layout()
fig.savefig(r"jsbsim_flight.png", dpi=120)
print("written jsbsim_flight.png")
