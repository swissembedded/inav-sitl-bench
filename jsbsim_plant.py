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

# JSBSim plant with the same sensor interface as inav-sitl-bench dynamics.PlaneModel.
# Feeds the PROVEN MSP_SIMULATOR/HITL injection path: acc = specific force (mg,
# +1000 on Z in level), gyro = deg/s * 16 body rates, baro from altitude.
import math
import sys

from dynamics import q_from_rpy, rotate_earth_to_body

import jsbsim

FT2M = 0.3048
G = 9.81


class JSBSimPlant:
    def __init__(self, model="aerobat3d", alt_ft=394, kts=45, dt=0.01):
        # 394 ft = 120 m: the legal RC ceiling; start the bench flights there
        # so replay altitudes look like real-world flying. Pass a higher
        # alt_ft explicitly for maneuvers that trade away more height.
        import os
        self.fdm = jsbsim.FGFDMExec(None)
        self.fdm.set_debug_level(0)
        here = os.path.dirname(os.path.abspath(__file__))
        local = os.path.join(here, "jsbsim", "aircraft", model)
        if os.path.isdir(local):   # repo-local aircraft (aerobat3d); else jsbsim built-ins (c172p)
            self.fdm.set_aircraft_path(os.path.join(here, "jsbsim", "aircraft"))
            self.fdm.set_engine_path(os.path.join(here, "jsbsim", "engine"))
        self.fdm.load_model(model)
        self.fdm.set_dt(dt)
        self.dt = dt
        f = self.fdm
        f["ic/h-sl-ft"] = alt_ft
        f["ic/vc-kts"] = kts
        f["ic/gamma-deg"] = 0
        # engine models (c172p): start running at IC. For external-force
        # models (aerobat3d) this just creates an unused property.
        f["propulsion/set-running"] = -1
        f.run_ic()
        self._v_prev = self._v_earth()
        self._a_earth = (0.0, 0.0, 0.0)

    # --- controls in stabilized-output units (-1..1, throttle 0..1) ---------
    def set_controls(self, ail, ele, rud, thr01):
        f = self.fdm
        f["fcs/aileron-cmd-norm"] = max(-1.0, min(1.0, ail))
        f["fcs/elevator-cmd-norm"] = max(-1.0, min(1.0, ele))
        f["fcs/rudder-cmd-norm"] = max(-1.0, min(1.0, rud))
        f["fcs/throttle-cmd-norm"] = max(0.0, min(1.0, thr01))

    BASE_DT = 0.001   # internal integration step; NEVER integrate coarser
                      # (40 ms steps blow up numerically -> NaN attitude)

    def step(self, dt=None):
        # advance sim time by dt using fixed fine substeps, so the coupling
        # rate (controller loop) is decoupled from the integrator step
        span = self.dt if dt is None else dt
        n = max(1, int(round(span / self.BASE_DT)))
        if abs(self.fdm.get_delta_t() - self.BASE_DT) > 1e-9:
            self.fdm.set_dt(self.BASE_DT)
        v0 = self._v_earth()
        for _ in range(n):
            self.fdm.run()
        v1 = self._v_earth()
        self._a_earth = tuple((b - a) / (n * self.BASE_DT) for a, b in zip(v0, v1))
        self._v_prev = v1

    # --- state ---------------------------------------------------------------
    def _v_earth(self):
        f = self.fdm   # earth frame: north, east, UP (bench convention)
        return (f["velocities/v-north-fps"] * FT2M,
                f["velocities/v-east-fps"] * FT2M,
                -f["velocities/v-down-fps"] * FT2M)

    def rpy(self):
        f = self.fdm
        return (f["attitude/phi-deg"], f["attitude/theta-deg"], f["attitude/psi-deg"])

    @property
    def q(self):
        r, p, y = self.rpy()
        return q_from_rpy(r, p, y)

    @property
    def z(self):
        return self.fdm["position/h-sl-ft"] * FT2M

    def ias_kts(self):
        return self.fdm["velocities/vc-kts"]

    def xy(self):
        f = self.fdm   # north/east in m relative to start
        if not hasattr(self, "_ll0"):
            self._ll0 = (f["position/lat-gc-deg"], f["position/long-gc-deg"])
        la0, lo0 = self._ll0
        import math as _m
        return ((f["position/lat-gc-deg"] - la0) * 111320.0,
                (f["position/long-gc-deg"] - lo0) * 111320.0 * _m.cos(_m.radians(la0)))

    def set_wind(self, north_ms=0.0, east_ms=0.0, down_ms=0.0):
        """Steady wind / gust in earth frame [m/s]; positive down = downdraft."""
        f = self.fdm
        f["atmosphere/wind-north-fps"] = north_ms / FT2M
        f["atmosphere/wind-east-fps"] = east_ms / FT2M
        f["atmosphere/wind-down-fps"] = down_ms / FT2M

    def gps(self):
        """GPS-fix injection so the FC's nav altitude estimate becomes trusted
        (navIsAltitudeEstimateTrusted) -- the figure altitude assist returns 0
        without it, so the plane would not hold altitude in the holds."""
        f = self.fdm
        vn, ve, vd = (f["velocities/v-north-fps"] * FT2M,
                      f["velocities/v-east-fps"] * FT2M,
                      f["velocities/v-down-fps"] * FT2M)
        return {
            "lat_e7": int(round(f["position/lat-gc-deg"] * 1e7)),
            "lon_e7": int(round(f["position/long-gc-deg"] * 1e7)),
            "alt_cm": int(round(self.z * 100)),
            "speed_cms": int(round(math.hypot(vn, ve) * 100)),
            "course_dd": int(round(math.degrees(math.atan2(ve, vn)) % 360.0 * 10)) % 3600,
            "vel_ned_cms": (int(round(vn * 100)), int(round(ve * 100)), int(round(vd * 100))),
        }

    # --- sensors (bench conventions) -----------------------------------------
    def gyro_dps16(self):
        f = self.fdm
        p = math.degrees(f["velocities/p-rad_sec"])
        qq = math.degrees(f["velocities/q-rad_sec"])
        r = math.degrees(f["velocities/r-rad_sec"])
        # saturate like the real IMU (int16 = +-2048 deg/s)
        return tuple(max(-32767, min(32767, int(round(v * 16)))) for v in (p, qq, r))

    def acc_mg(self):
        f_earth = (self._a_earth[0] / G,
                   self._a_earth[1] / G,
                   self._a_earth[2] / G + 1.0)
        f_body = rotate_earth_to_body(self.q, f_earth)
        return tuple(int(round(max(-16.0, min(16.0, a)) * 1000)) for a in f_body)

    def baro_pa(self):
        return int(round(101325.0 * math.exp(-self.z / 8434.0)))
