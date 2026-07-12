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

"""Independent verification of all quaternion/rotation math used in the
INAV orientation-hold work, against scipy.spatial.transform.Rotation
(externally validated reference implementation) and the published
aerospace ZYX (yaw-pitch-roll) convention.

INAV formulas are transcribed 1:1 from:
  src/main/common/quaternion.h  (quaternionMultiply, Conjugate,
                                 RotateVector, axisAngleToQuaternion,
                                 quaternionToAxisAngle)
  src/main/flight/imu.c         (imuComputeQuaternionFromRPY)
  src/main/flight/orientation_hold.c (reduced attitude error)
"""
import numpy as np
from scipy.spatial.transform import Rotation as R

rng = np.random.default_rng(42)
N = 2000
FAIL = []


def check(name, cond):
    if cond:
        print(f"PASS  {name}")
    else:
        print(f"FAIL  {name}")
        FAIL.append(name)


# ---- INAV formulas, transcribed 1:1 (w,x,y,z order) ----
def inav_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw])


def inav_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def inav_rotate(v, q):
    # quaternionRotateVector: q* (x) v (x) q
    p = inav_mul(inav_mul(inav_conj(q), np.array([0.0, *v])), q)
    return p[1:]


def inav_rotate_inv(v, q):
    # quaternionRotateVectorInv: q (x) v (x) q*
    p = inav_mul(inav_mul(q, np.array([0.0, *v])), inav_conj(q))
    return p[1:]


def inav_rpy_to_quat(r_deg, p_deg, y_deg):
    # imuComputeQuaternionFromRPY, exactly
    cR, sR = np.cos(np.radians(r_deg)/2), np.sin(np.radians(r_deg)/2)
    cP, sP = np.cos(np.radians(p_deg)/2), np.sin(np.radians(p_deg)/2)
    cY, sY = np.cos(np.radians(y_deg)/2), np.sin(np.radians(y_deg)/2)
    return np.array([
        cR*cP*cY + sR*sP*sY,
        sR*cP*cY - cR*sP*sY,
        cR*sP*cY + sR*cP*sY,
        cR*cP*sY - sR*sP*cY])


def inav_axis_angle_to_quat(axis, angle):
    # axisAngleToQuaternion: NOTE the negated axis
    s = np.sin(angle/2)
    return np.array([np.cos(angle/2), -axis[0]*s, -axis[1]*s, -axis[2]*s])


def inav_quat_to_axis_angle(q):
    # quaternionToAxisAngle
    angle = 2*np.arccos(np.clip(q[0], -1, 1))
    if angle > np.pi:
        angle -= 2*np.pi
    s = np.sqrt(max(0.0, 1 - q[0]*q[0]))
    if s > 1e-4:
        return q[1:]/s, angle
    return np.array([1.0, 0, 0]), 0.0


def reduced_attitude_error(q_est, q_tgt):
    # orientation_hold.c: shortest rotation between earth-up directions
    up_e = inav_rotate([0, 0, 1], q_est); up_e /= np.linalg.norm(up_e)
    up_t = inav_rotate([0, 0, 1], q_tgt); up_t /= np.linalg.norm(up_t)
    cross = np.cross(up_t, up_e)          # operand order = pidLevel sign convention
    cn = np.linalg.norm(cross)
    dot = float(np.dot(up_e, up_t))
    ang = np.arctan2(cn, dot)
    if cn > 1e-6:
        return cross/cn * ang
    return np.zeros(3) if dot > 0 else None  # 180deg handled separately


def scipy_q(q_wxyz):
    """INAV (w,x,y,z) -> scipy Rotation"""
    w, x, y, z = q_wxyz
    return R.from_quat([x, y, z, w])


# ============================================================
# A. Euler(ZYX) -> quaternion vs scipy (published aerospace convention)
ok = True
for _ in range(N):
    r_, p_, y_ = rng.uniform(-180, 180), rng.uniform(-89, 89), rng.uniform(-180, 180)
    qi = inav_rpy_to_quat(r_, p_, y_)
    qs = R.from_euler('ZYX', [y_, p_, r_], degrees=True).as_quat()  # [x,y,z,w]
    qs = np.array([qs[3], qs[0], qs[1], qs[2]])
    if min(np.abs(qi - qs).max(), np.abs(qi + qs).max()) > 1e-9:
        ok = False
        break
check("A  imuComputeQuaternionFromRPY == scipy intrinsic ZYX (yaw-pitch-roll)", ok)

# B. quaternionRotateVector semantics vs scipy
ok = okc = True
for _ in range(N):
    q = inav_rpy_to_quat(rng.uniform(-180, 180), rng.uniform(-89, 89), rng.uniform(-180, 180))
    v = rng.normal(size=3)
    rot = scipy_q(q)
    if np.abs(inav_rotate(v, q) - rot.apply(v, inverse=True)).max() > 1e-9:
        ok = False
    if np.abs(inav_rotate_inv(v, q) - rot.apply(v)).max() > 1e-9:
        okc = False
check("B1 quaternionRotateVector(v,q) == R^-1 v (earth->body, passive)", ok)
check("B2 quaternionRotateVectorInv(v,q) == R v (body->earth, active)", okc)

# B3 earth-up in body must be yaw-invariant (physical requirement)
ok = True
for _ in range(N // 4):
    r_, p_ = rng.uniform(-180, 180), rng.uniform(-89, 89)
    u0 = inav_rotate([0, 0, 1], inav_rpy_to_quat(r_, p_, 0))
    u1 = inav_rotate([0, 0, 1], inav_rpy_to_quat(r_, p_, rng.uniform(-180, 180)))
    if np.abs(u0 - u1).max() > 1e-9:
        ok = False
check("B3 earthUpInBodyFrame is yaw-invariant", ok)

# C. Reduced attitude error vs scipy ground truth
ok_ang = ok_axis = ok_small = True
for _ in range(N):
    qe = inav_rpy_to_quat(rng.uniform(-180, 180), rng.uniform(-89, 89), rng.uniform(-180, 180))
    qt = inav_rpy_to_quat(rng.uniform(-180, 180), rng.uniform(-89, 89), 0)
    err = reduced_attitude_error(qe, qt)
    if err is None:
        continue
    up_e = scipy_q(qe).apply([0, 0, 1], inverse=True)
    up_t = scipy_q(qt).apply([0, 0, 1], inverse=True)
    ang_ref = np.arccos(np.clip(np.dot(up_e, up_t), -1, 1))
    if abs(np.linalg.norm(err) - ang_ref) > 1e-7:
        ok_ang = False
    # applying the error rotation to up_est must yield up_tgt (Rodrigues via scipy)
    if np.linalg.norm(err) > 1e-8:
        corr = R.from_rotvec(-err)   # sign: err = target - attitude (pidLevel convention)
        if np.abs(corr.apply(up_e) - up_t).max() > 1e-6:
            ok_axis = False
check("C1 reduced-attitude error magnitude == angle(up_est, up_tgt) [scipy]", ok_ang)
check("C2 error axis: rotating up_est by -err yields up_tgt exactly [scipy]", ok_axis)

# C3 small-angle equivalence with Euler pidLevel error (doc claim)
for _ in range(N // 4):
    r_, p_ = rng.uniform(-8, 8), rng.uniform(-8, 8)
    err = reduced_attitude_error(inav_rpy_to_quat(r_, p_, rng.uniform(-180, 180)),
                                 inav_rpy_to_quat(0, 0, 0))
    if abs(np.degrees(err[0]) - (-r_)) > 0.15 or abs(np.degrees(err[1]) - (-p_)) > 0.15:
        ok_small = False
check("C3 small-angle: err == (target - attitude) per axis, any heading", ok_small)

# C4 exact single-axis equivalence at large angles
ok = True
for a in range(-170, 171, 5):
    err = reduced_attitude_error(inav_rpy_to_quat(a, 0, 0), inav_rpy_to_quat(0, 0, 0))
    if abs(np.degrees(err[0]) + a) > 1e-6 or abs(err[1]) > 1e-9:
        ok = False
check("C4 exact single-axis: roll sweep +-170 vs level", ok)

# D. Bench dynamics functions (same algebra as firmware) + nose fix
ok = True
for _ in range(N // 2):
    q = inav_rpy_to_quat(rng.uniform(-180, 180), rng.uniform(-89, 89), rng.uniform(-180, 180))
    # nose fix: earth_to_body(qconj(q), x) must equal R x (body->earth active)
    nose = inav_rotate([1, 0, 0], inav_conj(q))
    if np.abs(nose - scipy_q(q).apply([1, 0, 0])).max() > 1e-9:
        ok = False
check("D1 nose fix: rotate(x, conj(q)) == R x (body->earth) [scipy]", ok)

# D2 nose elevation yaw-invariant and == -sin(pitch) in this convention
ok = True
for _ in range(N // 4):
    p_ = rng.uniform(-89, 89)
    nz = inav_rotate([1, 0, 0], inav_conj(inav_rpy_to_quat(rng.uniform(-180, 180), p_, rng.uniform(-180, 180))))[2]
    if abs(-nz - np.sin(np.radians(p_))) > 1e-9:
        ok = False
check("D2 nose elevation == asin(-nose_z), yaw-invariant, matches pitch", ok)

# E. axisAngleToQuaternion / quaternionToAxisAngle are NOT mutual inverses
axis = np.array([0.36, 0.48, 0.8])
q_std = np.array([np.cos(0.5), *(axis*np.sin(0.5))])       # published standard
q_inav = inav_axis_angle_to_quat(axis, 1.0)
ax_back, ang_back = inav_quat_to_axis_angle(q_inav)
check("E1 INAV axisAngleToQuaternion == CONJUGATE of the standard form",
      np.abs(q_inav - inav_conj(q_std)).max() < 1e-12)
check("E2 round-trip toAxisAngle(axisAngleTo(a)) negates the axis (documented trap)",
      np.abs(ax_back + axis).max() < 1e-9 and abs(ang_back - 1.0) < 1e-9)
check("E3 quaternionToAxisAngle alone matches scipy as_rotvec",
      np.abs(np.array(inav_quat_to_axis_angle(q_std)[0]) * inav_quat_to_axis_angle(q_std)[1]
             - scipy_q(q_std).as_rotvec()).max() < 1e-7)

# F. Figure identities
u1 = inav_rotate([0, 0, 1], inav_rpy_to_quat(180, 168, 0))
u2 = inav_rotate([0, 0, 1], inav_rpy_to_quat(0, 12, 0))
check("F1 Immelmann trim identity: up(rp(180,168)) == up(rp(0,12))",
      np.abs(u1 - u2).max() < 1e-9)
q_seq = inav_rpy_to_quat(180, 180, 0)
check("F2 rp(180,180) is upright (Immelmann end): up == e_z",
      np.abs(inav_rotate([0, 0, 1], q_seq) - np.array([0, 0, 1])).max() < 1e-9)
# antipode: level vs inverted = exactly 180
err_mag = None
up_e = inav_rotate([0, 0, 1], inav_rpy_to_quat(0, 0, 0))
up_t = inav_rotate([0, 0, 1], inav_rpy_to_quat(180, 0, 0))
check("F3 level vs inverted: up vectors exactly antiparallel (180 deg case)",
      np.abs(up_e + up_t).max() < 1e-12)

# G. Full attitude error (figure line-hold): q_err = conj(q_est) (x) q_tgt,
#    rotation vector in the estimated BODY frame via quaternionToAxisAngle
def full_attitude_error(q_est, q_tgt):
    q_err = inav_mul(inav_conj(q_est), q_tgt)
    ax, ang = inav_quat_to_axis_angle(q_err)
    return np.array(ax) * ang


def quat_from_rotvec(rv):
    # quatFromRotVecDeg (standard form, NOT axisAngleToQuaternion)
    ang = np.linalg.norm(rv)
    if ang < 1e-12:
        return np.array([1.0, 0, 0, 0])
    ax = np.array(rv) / ang
    return np.array([np.cos(ang / 2), *(ax * np.sin(ang / 2))])


# G0 quaternion multiply composes like rotation matrices
ok = True
for _ in range(N // 4):
    qa = inav_rpy_to_quat(*rng.uniform(-170, 170, 3))
    qb = inav_rpy_to_quat(*rng.uniform(-170, 170, 3))
    Rab = scipy_q(inav_mul(qa, qb)).as_matrix()
    if np.abs(Rab - scipy_q(qa).as_matrix() @ scipy_q(qb).as_matrix()).max() > 1e-9:
        ok = False
        break
check("G0 inav_mul(a,b) composes as R(a) @ R(b)", ok)

# G1 full error == scipy body-frame rotvec (relative angle < 180)
ok = True
for _ in range(N):
    qe = inav_rpy_to_quat(*rng.uniform(-170, 170, 3))
    rv = rng.uniform(-1.0, 1.0, 3)
    rv = rv / np.linalg.norm(rv) * rng.uniform(0.5, 170.0)
    qt = inav_mul(qe, quat_from_rotvec(np.radians(rv)))
    err = np.degrees(full_attitude_error(qe, qt))
    ref = np.degrees((scipy_q(qe).inv() * scipy_q(qt)).as_rotvec())
    if np.abs(err - ref).max() > 1e-5:
        ok = False
        break
check("G1 full_attitude_error == scipy (R_est^-1 R_tgt).as_rotvec, body frame", ok)

# G2 sign pin: target rolled +10 deg in the body frame -> err = (+10, 0, 0)
qe = inav_rpy_to_quat(20.0, -35.0, 140.0)
qt = inav_mul(qe, quat_from_rotvec(np.radians([10.0, 0, 0])))
err = np.degrees(full_attitude_error(qe, qt))
check("G2 body-roll offset +10 -> err (+10, 0, 0) (pidLevel sign)",
      np.abs(err - np.array([10.0, 0, 0])).max() < 1e-6)

# G3 exact relationship to the reduced error:
#    - a pure twist about the body-frame up axis is invisible to the reduced
#      error (heading-free by construction) and comes out of the full error
#      as exactly psi * up_body -- this is the component the line-hold adds
#    - a rotation about any axis perpendicular to body-up leaves both errors
#      identical: the full error changes nothing for pure tilt regulation
ok_twist = True
ok_tilt = True
for _ in range(N // 4):
    qe = inav_rpy_to_quat(*rng.uniform(-170, 170, 3))
    up_b = np.array(inav_rotate([0, 0, 1], qe))
    psi = np.radians(rng.uniform(-90, 90))
    qt = inav_mul(qe, quat_from_rotvec(psi * up_b))
    red = reduced_attitude_error(qe, qt)
    ful = full_attitude_error(qe, qt)
    if np.linalg.norm(red) > 1e-9 or np.abs(ful - psi * up_b).max() > 1e-9:
        ok_twist = False
        break
    ax = np.cross(up_b, rng.uniform(-1.0, 1.0, 3))
    ax /= np.linalg.norm(ax)
    th = np.radians(rng.uniform(1.0, 60.0))
    qt2 = inav_mul(qe, quat_from_rotvec(th * ax))
    if np.abs(reduced_attitude_error(qe, qt2) - full_attitude_error(qe, qt2)).max() > 1e-9:
        ok_tilt = False
        break
check("G3a pure twist about body-up: reduced == 0, full == psi * up_body", ok_twist)
check("G3b rotation about axis perpendicular to body-up: full == reduced", ok_tilt)

# G4 yaw-anchored figure target: q_yaw(psi) (x) TargetFromRP(r,p) == rpy(r,p,psi)
ok = True
for _ in range(N // 4):
    r_, p_, psi = rng.uniform(-170, 170), rng.uniform(-85, 85), rng.uniform(-170, 170)
    h = np.radians(psi) / 2
    q_yaw = np.array([np.cos(h), 0, 0, np.sin(h)])
    q = inav_mul(q_yaw, inav_rpy_to_quat(r_, p_, 0.0))
    qr = inav_rpy_to_quat(r_, p_, psi)
    if min(np.abs(q - qr).max(), np.abs(q + qr).max()) > 1e-9:
        ok = False
        break
check("G4 q_yaw(psi) (x) TargetFromRP(r,p) == rpy(r,p,psi) (anchor composition)", ok)

print()
print("ALL PASS" if not FAIL else f"{len(FAIL)} FAILURES: {FAIL}")
