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

"""Rigid-body attitude dynamics + consistent sensor synthesis.

Implements the gyro/acc/attitude consistency condition:
    gyro(t) = 2 * vec(q^-1 (x) qdot)      (body rates)
    acc(t)  = q^-1 (x) [0,0,g] (x) q      (quasistatic: gravity only)

Quaternion convention identical to INAV's imuComputeQuaternionFromRPY
(q maps body -> earth, earth Z up, ZYX Euler).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field


Quat = tuple[float, float, float, float]  # w, x, y, z


def qmul(a: Quat, b: Quat) -> Quat:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def qconj(q: Quat) -> Quat:
    return (q[0], -q[1], -q[2], -q[3])


def qnorm(q: Quat) -> Quat:
    n = math.sqrt(sum(c * c for c in q))
    return tuple(c / n for c in q)  # type: ignore


def q_from_rpy(roll_deg: float, pitch_deg: float, yaw_deg: float) -> Quat:
    cr = math.cos(math.radians(roll_deg) / 2); sr = math.sin(math.radians(roll_deg) / 2)
    cp = math.cos(math.radians(pitch_deg) / 2); sp = math.sin(math.radians(pitch_deg) / 2)
    cy = math.cos(math.radians(yaw_deg) / 2); sy = math.sin(math.radians(yaw_deg) / 2)
    return (cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy)


def rotate_earth_to_body(q: Quat, v: tuple[float, float, float]) -> tuple[float, float, float]:
    """v_body = q^-1 (x) v_earth (x) q  for q mapping body->earth."""
    p = qmul(qmul(qconj(q), (0.0, *v)), q)
    return (p[1], p[2], p[3])


def rotate_body_to_earth(q: Quat, v: tuple[float, float, float]) -> tuple[float, float, float]:
    p = qmul(qmul(q, (0.0, *v)), qconj(q))
    return (p[1], p[2], p[3])


def tilt_error_deg(q: Quat, q_target: Quat) -> float:
    """Heading-free attitude error: angle between the earth-up directions in
    the body frame (reduced attitude, same formulation as the firmware).
    A swing-twist decomposition about earth Z is degenerate near inverted
    (w^2+z^2 ~ 0 for every heading) and must not be used here."""
    u = rotate_earth_to_body(q, (0.0, 0.0, 1.0))
    ut = rotate_earth_to_body(q_target, (0.0, 0.0, 1.0))
    nu = math.sqrt(sum(c * c for c in u))
    nt = math.sqrt(sum(c * c for c in ut))
    dot = sum(a * b for a, b in zip(u, ut)) / (nu * nt)
    return math.degrees(math.acos(max(-1.0, min(1.0, dot))))


@dataclass
class PlaneModel:
    """Very small foam plane, torque = K*surface - D*omega, per body axis.

    Not aerodynamically faithful -- just a stable, controllable plant so the
    attitude loop can be exercised end to end.
    """
    inertia: tuple[float, float, float] = (0.02, 0.03, 0.04)     # kg m^2
    surface_torque: tuple[float, float, float] = (0.6, 0.6, 0.3)  # Nm at full deflection
    damping: tuple[float, float, float] = (0.05, 0.06, 0.08)      # Nm/(rad/s)
    q: Quat = field(default_factory=lambda: (1.0, 0.0, 0.0, 0.0))
    omega: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])  # rad/s body
    airspeed: float = 15.0   # m/s, constant (kinematic altitude model)
    sink_rate: float = 1.5   # m/s sink when the wings carry no lift
                             # (keep compensable by ~12 deg of flight path at
                             # 15 m/s: inverted sinks at 2x this value)
    z: float = 0.0           # m above home
    x: float = 0.0           # m north of home
    y: float = 0.0           # m east of home
    _omega_meas: tuple = (0.0, 0.0, 0.0)   # measured rates incl. turn coordination
    _v_prev: tuple | None = None
    _a_earth: tuple = (0.0, 0.0, 0.0)
    _vz_hover: float = 0.0   # thrust response lags (motor/prop time constant)

    def set_attitude(self, roll_deg: float, pitch_deg: float, yaw_deg: float = 0.0):
        self.q = q_from_rpy(roll_deg, pitch_deg, yaw_deg)
        self.omega = [0.0, 0.0, 0.0]
        self._v_prev = None   # teleport: no acceleration spike
        self._a_earth = (0.0, 0.0, 0.0)

    def step(self, surfaces: tuple[float, float, float], dt: float, substeps: int = 10,
             throttle01: float | None = None):
        # coordinated turn: bank tilts the lift vector and rotates the course
        # about the earth vertical; fed through omega so the gyro stream stays
        # consistent (doc consistency condition). Fades away from wings-level.
        w, xq, yq, zq = self.q
        bank = math.atan2(2 * (w * xq + yq * zq), 1 - 2 * (xq * xq + yq * yq))
        lift_align_pre = rotate_earth_to_body(self.q, (0.0, 0.0, 1.0))[2]
        bank = max(-1.0, min(1.0, bank))   # rad, clamp +-57 deg for tan
        psidot = 9.81 / self.airspeed * math.tan(bank) * max(0.0, lift_align_pre)
        up_body = rotate_earth_to_body(self.q, (0.0, 0.0, 1.0))
        turn_omega = tuple(psidot * c for c in up_body)

        h = dt / substeps
        for _ in range(substeps):
            tau = [self.surface_torque[i] * surfaces[i] - self.damping[i] * self.omega[i]
                   for i in range(3)]
            for i in range(3):
                self.omega[i] += h * tau[i] / self.inertia[i]
            omega_eff = tuple(self.omega[i] + turn_omega[i] for i in range(3))
            # qdot = 0.5 * q (x) (0, omega)
            dq = qmul(self.q, (0.0, *omega_eff))
            self.q = qnorm(tuple(self.q[i] + 0.5 * h * dq[i] for i in range(4)))  # type: ignore
        self._omega_meas = tuple(self.omega[i] + turn_omega[i] for i in range(3))
        # kinematic climb: velocity along body X at constant airspeed.
        # NOTE quaternion convention: rotate_earth_to_body() is the mapping
        # that yields body-X in EARTH coordinates here (+z when the FC shows
        # positive = nose-up pitch; verified against the SITL AHRS). The
        # naming flip is inherited from INAV's quaternionRotateVector.
        # Nose direction in the earth frame: the inverse mapping of the
        # (FC-validated) gravity synthesis, i.e. earth_to_body with the
        # conjugated quaternion. The old form rotate_earth_to_body(q, x)
        # was only correct at yaw 0 and flipped the climb sign at heading
        # 180 - the plane sank nose-up after every Immelmann while the FC
        # was right all along. z is negated into the up-positive frame
        # (yaw-invariant elevation, verified against the FC).
        n = rotate_earth_to_body(qconj(self.q), (1.0, 0.0, 0.0))
        dir_earth = (n[0], n[1], -n[2])
        # wings only lift along +body-Z: away from wings-level the plane
        # sinks (knife: full sink, inverted: doubled without push). The z
        # component of body-up in earth is transpose-invariant, so the
        # rotate-direction ambiguity does not matter here.
        lift_alignment = rotate_earth_to_body(self.q, (0.0, 0.0, 1.0))[2]
        sink = self.sink_rate * (1.0 - max(0.0, lift_alignment))
        vz = self.airspeed * dir_earth[2] - sink
        if throttle01 is not None:
            # thrust-borne regime: near nose-vertical the climb rate follows
            # the throttle around a hover point instead of the wing kinematics
            # blend to thrust-borne early: pulling to the vertical bleeds
            # airspeed, the wings stop carrying well before the zenith
            elev = math.degrees(math.asin(max(-1.0, min(1.0, dir_earth[2]))))
            f = max(0.0, min(1.0, (elev - 25.0) / 30.0))
            vz_cmd = 14.0 * (throttle01 - 0.55)
            self._vz_hover += (vz_cmd - self._vz_hover) * min(1.0, dt / 0.5)
            vz = (1.0 - f) * vz + f * self._vz_hover
        v_new = (self.airspeed * dir_earth[0],
                 self.airspeed * dir_earth[1],
                 vz)
        self.x += v_new[0] * dt
        self.y += v_new[1] * dt
        self.z += v_new[2] * dt
        # maneuver acceleration for the specific-force accelerometer model;
        # without it the FC's INS believes vz = 0 forever, the baro diverges
        # from the INS and the estimator declares the altitude untrusted
        if self._v_prev is not None:
            self._a_earth = tuple((v_new[i] - self._v_prev[i]) / dt for i in range(3))
        self._v_prev = v_new

    # ---- sensor synthesis (doc consistency equations) ----
    def gyro_dps16(self) -> tuple[int, int, int]:
        w = self._omega_meas if any(self._omega_meas) else tuple(self.omega)
        return tuple(int(round(math.degrees(v) * 16)) for v in w)  # type: ignore

    def gps(self) -> dict:
        """GPS injection data (NEU home at lat 47 / lon 8.5)."""
        lat0, lon0 = 47.0, 8.5
        lat = lat0 + self.x / 111320.0
        lon = lon0 + self.y / (111320.0 * math.cos(math.radians(lat0)))
        if self._v_prev is not None:
            vn, ve, vz = self._v_prev
        else:
            vn = ve = vz = 0.0
        course = math.degrees(math.atan2(ve, vn)) % 360.0
        return {
            "lat_e7": int(round(lat * 1e7)),
            "lon_e7": int(round(lon * 1e7)),
            "alt_cm": int(round(self.z * 100)),
            "speed_cms": int(round(math.hypot(vn, ve) * 100)),
            "course_dd": int(round(course * 10)) % 3600,
            "vel_ned_cms": (int(round(vn * 100)), int(round(ve * 100)), int(round(-vz * 100))),
        }

    def acc_mg(self) -> tuple[int, int, int]:
        # specific force f = a_earth + g_up, expressed in the body frame with
        # the same (validated) mapping used for gravity; linearity keeps the
        # sign conventions consistent
        G = 9.81
        f_earth = (self._a_earth[0] / G,
                   self._a_earth[1] / G,
                   self._a_earth[2] / G + 1.0)
        f_body = rotate_earth_to_body(self.q, f_earth)
        # clamp to a realistic sensor range (+-16 g, like the real IMU)
        return tuple(int(round(max(-16.0, min(16.0, a)) * 1000)) for a in f_body)  # type: ignore

    def baro_pa(self) -> int:
        return int(round(101325.0 * math.exp(-self.z / 8434.0)))

    def pitch_deg(self) -> float:
        w, x, y, z = self.q
        return math.degrees(math.asin(max(-1.0, min(1.0, 2.0 * (w * y - x * z)))))

    def nose_elevation_deg(self) -> float:
        """True nose elevation above the horizon (yaw-invariant)."""
        n = rotate_earth_to_body(qconj(self.q), (1.0, 0.0, 0.0))
        return math.degrees(math.asin(max(-1.0, min(1.0, -n[2]))))
