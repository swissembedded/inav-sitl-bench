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

"""Level-1 controller-numerics test.

Injects (q_est, q_target) pairs into the FC via
MSP2_INAV_ORIENTATION_HOLD_TEST (pure computation on the target MCU's
float32) and compares the returned error vector against a float64 NumPy
reference of the same reduced-attitude formulation. Covers the
singularity checklist: signs, yaw invariance, sweep through pitch 90,
the 180-degree antipode and the near-inverted degeneracy regression.

Run against SITL:      python level1_test.py
Run against a real FC: python level1_test.py COM7   (any attitude, safe:
no state is touched; do this with props OFF anyway)
"""
from __future__ import annotations

import struct
import sys

import numpy as np

from msp import MspClient

MSP2_INAV_ORIENTATION_HOLD_TEST = 0x2242


def q_from_rpy(r, p, y):
    cr, sr = np.cos(np.radians(r) / 2), np.sin(np.radians(r) / 2)
    cp, sp = np.cos(np.radians(p) / 2), np.sin(np.radians(p) / 2)
    cy, sy = np.cos(np.radians(y) / 2), np.sin(np.radians(y) / 2)
    return np.array([cr * cp * cy + sr * sp * sy,
                     sr * cp * cy - cr * sp * sy,
                     cr * sp * cy + sr * cp * sy,
                     cr * cp * sy - sr * sp * cy])


def qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([aw * bw - ax * bx - ay * by - az * bz,
                     aw * bx + ax * bw + ay * bz - az * by,
                     aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw])


def up_in_body(q):
    qc = np.array([q[0], -q[1], -q[2], -q[3]])
    p = qmul(qmul(qc, np.array([0, 0, 0, 1.0])), q)
    v = p[1:]
    return v / np.linalg.norm(v)


def reference_err_deg(q_est, q_tgt):
    ue, ut = up_in_body(q_est), up_in_body(q_tgt)
    cross = np.cross(ut, ue)
    cn = np.linalg.norm(cross)
    ang = np.arctan2(cn, float(np.dot(ue, ut)))
    if cn > 1e-6:
        return np.degrees(cross / cn * ang)
    return np.zeros(3) if np.dot(ue, ut) > 0 else None  # 180 deg: axis is FC's choice


def fc_eval(msp, q_est, q_tgt):
    payload = struct.pack("<8f", *q_est, *q_tgt)
    reply = msp.request(MSP2_INAV_ORIENTATION_HOLD_TEST, payload)
    vals = struct.unpack("<6f", reply)
    return np.array(vals[:3]), np.array(vals[3:])


def main():
    host = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
    msp = MspClient(host)
    n_pass = n_fail = 0
    worst = 0.0

    def check(name, q_est, q_tgt, tol_deg=0.02):
        nonlocal n_pass, n_fail, worst
        err_fc, rate_fc = fc_eval(msp, q_est, q_tgt)
        ref = reference_err_deg(q_est, q_tgt)
        if not np.all(np.isfinite(err_fc)) or not np.all(np.isfinite(rate_fc)):
            n_fail += 1
            print(f"FAIL {name}: non-finite {err_fc} {rate_fc}")
            return
        if ref is None:  # exact 180: magnitude must match, axis is free
            dev = abs(np.linalg.norm(err_fc) - 180.0)
        else:
            dev = float(np.abs(err_fc - ref).max())
        worst = max(worst, dev)
        if dev <= tol_deg:
            n_pass += 1
        else:
            n_fail += 1
            print(f"FAIL {name}: fc {err_fc} ref {ref} (dev {dev:.4f} deg)")

    # signs (pidLevel convention)
    check("pitch +5 vs level", q_from_rpy(0, 5, 0), q_from_rpy(0, 0, 0))
    check("roll +5 vs level", q_from_rpy(5, 0, 0), q_from_rpy(0, 0, 0))
    # yaw invariance
    for y in (0, 45, 120, 180, 250):
        check(f"tilt(5,-3) yaw {y}", q_from_rpy(5, -3, y), q_from_rpy(0, 0, 0))
    # sweep through the Euler singularity
    for p in np.arange(80, 101, 2.5):
        check(f"prophang sweep pitch {p}", q_from_rpy(0, p, 0), q_from_rpy(0, 90, 0))
    # antipode
    check("roll 179 vs inverted", q_from_rpy(179, 0, 0), q_from_rpy(180, 0, 0))
    check("roll -179 vs inverted", q_from_rpy(-179, 0, 0), q_from_rpy(180, 0, 0))
    check("level vs inverted (exact 180)", q_from_rpy(0, 0, 0), q_from_rpy(180, 0, 0), tol_deg=0.1)
    # near-inverted degeneracy regression (the old swing-twist trap)
    check("inverted -179.9/0.4 vs inverted", q_from_rpy(-179.9, 0.4, 0), q_from_rpy(180, 0, 0))
    check("inverted heading 60 vs inverted", q_from_rpy(180, 0, 60), q_from_rpy(180, 0, 0))
    # norm drift in
    check("denormalized est (x1.1)", 1.1 * q_from_rpy(10, 5, 0), q_from_rpy(0, 0, 0), tol_deg=0.05)
    # random grid
    rng = np.random.default_rng(7)
    for i in range(60):
        qe = q_from_rpy(rng.uniform(-180, 180), rng.uniform(-85, 85), rng.uniform(-180, 180))
        qt = q_from_rpy(rng.uniform(-180, 180), rng.uniform(-85, 85), 0)
        check(f"random {i}", qe, qt, tol_deg=0.05)

    msp.close()
    print(f"\n{n_pass} passed, {n_fail} failed, worst float32-vs-float64 deviation {worst:.4f} deg")
    sys.exit(1 if n_fail else 0)


if __name__ == "__main__":
    main()
