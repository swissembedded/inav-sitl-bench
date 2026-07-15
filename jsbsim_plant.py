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
    # Servo rate limits (best aerobatic HV servos, unloaded, 8.4 V):
    # aileron/elevator ~0.06 s per 60 deg, rudder ~0.10 s per 60 deg (torque
    # over speed there). With a +-60 deg 3D throw = full normalized range,
    # full travel takes 2x that: the surfaces physically cannot follow a
    # faster command, so the plant must not either - a controller that only
    # works with instant surfaces would be lying to us.
    SERVO_SLEW_AIL_ELE = 1.0 / 0.06   # normalized units per second
    SERVO_SLEW_RUD     = 1.0 / 0.10

    def set_controls(self, ail, ele, rud, thr01):
        # commands are TARGETS; step() slews the servo positions toward them
        # with the real integration span (set_controls has no honest dt)
        self._servo_cmd = (max(-1.0, min(1.0, ail)),
                           max(-1.0, min(1.0, ele)),
                           max(-1.0, min(1.0, rud)))
        self.fdm["fcs/throttle-cmd-norm"] = max(0.0, min(1.0, thr01))

    def _servo_step(self, span):
        if not hasattr(self, "_servo_pos"):
            self._servo_pos = list(getattr(self, "_servo_cmd", (0.0, 0.0, 0.0)))
        cmd = getattr(self, "_servo_cmd", (0.0, 0.0, 0.0))
        for i, slew in enumerate((self.SERVO_SLEW_AIL_ELE,
                                  self.SERVO_SLEW_AIL_ELE,
                                  self.SERVO_SLEW_RUD)):
            step = slew * span
            self._servo_pos[i] += max(-step, min(step, cmd[i] - self._servo_pos[i]))
        f = self.fdm
        f["fcs/aileron-cmd-norm"] = self._servo_pos[0]
        f["fcs/elevator-cmd-norm"] = self._servo_pos[1]
        f["fcs/rudder-cmd-norm"] = self._servo_pos[2]

    BASE_DT = 0.001   # internal integration step; NEVER integrate coarser
                      # (40 ms steps blow up numerically -> NaN attitude)

    def step(self, dt=None):
        # advance sim time by dt using fixed fine substeps, so the coupling
        # rate (controller loop) is decoupled from the integrator step
        span = self.dt if dt is None else dt
        self._servo_step(span)
        self._tvc_step(span)
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

    # The nozzle actuator is a servo too: +-15 deg full deflection driven by
    # the same 0.06 s/60 deg class as the surfaces -> 15 deg (1.0 norm) in
    # 0.015 s, full sweep -1..+1 in 0.03 s. Fast, but not instant - and the
    # TVC hover stands on exactly this actuator.
    SERVO_SLEW_TVC = 1.0 / 0.015  # normalized units per second

    def set_tvc(self, pitch_norm=0.0, yaw_norm=0.0):
        """Vectored-nozzle deflection targets, -1..1 (funjet); slewed in
        step() like the surface servos; no-op on airframes without TVC."""
        self._tvc_cmd = (max(-1.0, min(1.0, pitch_norm)),
                         max(-1.0, min(1.0, yaw_norm)))

    def _tvc_step(self, span):
        if not hasattr(self, "_tvc_pos"):
            self._tvc_pos = list(getattr(self, "_tvc_cmd", (0.0, 0.0)))
        cmd = getattr(self, "_tvc_cmd", (0.0, 0.0))
        step = self.SERVO_SLEW_TVC * span
        for i in range(2):
            self._tvc_pos[i] += max(-step, min(step, cmd[i] - self._tvc_pos[i]))
        self.fdm["fcs/tvc-pitch-norm"] = self._tvc_pos[0]
        self.fdm["fcs/tvc-yaw-norm"] = self._tvc_pos[1]

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

    def set_imu_offset(self, x_m=0.0, y_m=0.0, z_m=0.0):
        """IMU lever arm from the CG in body frame [m]. A sensor off the CG
        additionally measures the centripetal term w x (w x r) plus the
        angular-acceleration term alpha x r -- constant in the body frame
        during a steady spin, i.e. exactly the false-down pull a CG-mounted
        model cannot show."""
        self._imu_r = (x_m, y_m, z_m)
        self._imu_w_prev = None

    def _imu_lever_arm_g(self):
        r = getattr(self, "_imu_r", (0.0, 0.0, 0.0))
        if r == (0.0, 0.0, 0.0):
            return (0.0, 0.0, 0.0)
        f = self.fdm
        w = (f["velocities/p-rad_sec"], f["velocities/q-rad_sec"],
             f["velocities/r-rad_sec"])
        wxr = (w[1] * r[2] - w[2] * r[1],
               w[2] * r[0] - w[0] * r[2],
               w[0] * r[1] - w[1] * r[0])
        a = [w[1] * wxr[2] - w[2] * wxr[1],
             w[2] * wxr[0] - w[0] * wxr[2],
             w[0] * wxr[1] - w[1] * wxr[0]]
        wp = getattr(self, "_imu_w_prev", None)
        if wp is not None:
            alpha = tuple((b - c) / self.BASE_DT for b, c in zip(w, wp))
            a[0] += alpha[1] * r[2] - alpha[2] * r[1]
            a[1] += alpha[2] * r[0] - alpha[0] * r[2]
            a[2] += alpha[0] * r[1] - alpha[1] * r[0]
        self._imu_w_prev = w
        return (a[0] / G, a[1] / G, a[2] / G)

    def acc_mg(self):
        f_earth = (self._a_earth[0] / G,
                   self._a_earth[1] / G,
                   self._a_earth[2] / G + 1.0)
        f_body = rotate_earth_to_body(self.q, f_earth)
        lever = self._imu_lever_arm_g()
        f_body = tuple(fb + lv for fb, lv in zip(f_body, lever))
        return tuple(int(round(max(-16.0, min(16.0, a)) * 1000)) for a in f_body)

    def baro_pa(self):
        return int(round(101325.0 * math.exp(-self.z / 8434.0)))
