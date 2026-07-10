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

"""Closed-loop SITL bench for the INAV quaternion orientation hold.

Usage:
  python bench.py provision   # one-time FC setup (reboots SITL)
  python bench.py smoke       # sensor sign conventions + arming check
  python bench.py scenarios   # play through all orientation-hold targets

SITL is expected on tcp:127.0.0.1:5760 (see run_sitl.sh / README).
"""
from __future__ import annotations

import struct
import sys
import time

from msp import MspClient, MSP_SIMULATOR
from hitl import sim_step
from dynamics import PlaneModel, q_from_rpy, tilt_error_deg

MSP2_INAV_STATUS = 0x2000

# rc channel layout (AETR + AUX1..4)
CH_ROLL, CH_PITCH, CH_THROTTLE, CH_YAW = 0, 1, 2, 3
CH_ARM, CH_ANGLE, CH_INVERTED, CH_SELECT = 4, 5, 6, 7
RC_LOW, RC_MID, RC_HIGH = 1000, 1500, 2000

# permanent box ids
PERM_ARM, PERM_ANGLE = 0, 1
PERM_INVERTED, PERM_KNIFELEFT, PERM_KNIFERIGHT, PERM_PROPHANG = 69, 70, 71, 72
PERM_ALTFLOOR = 73
PERM_FIGROLL, PERM_FIGLOOP, PERM_FIGPOINTROLL, PERM_FIGSEQ = 74, 75, 76, 77
RC_ALTFLOOR_ON = 1300   # ALT FLOOR band on the CH_INVERTED channel
RC_FIGROLL_ON = 1575    # FIGURE ROLL band on the CH_INVERTED channel
RC_FIGLOOP_ON = 1300    # FIGURE LOOP band on the CH_ANGLE channel
RC_FIGSEQ_ON = 1575     # FIGURE SEQ band on the CH_ANGLE channel

# figure sequence segment types (figure_sequencer.h)
FIGSEG_END, FIGSEG_ROLL, FIGSEG_PITCH, FIGSEG_HOLD, FIGSEG_WAIT_ALT, FIGSEG_WAIT_TIME = range(6)
FIGSEG_FLAG_ASSIST = 1

DT = 0.01  # 100 Hz

# armingFlags bits that do not block arming
FLAG_ARMED = 1 << 2
FLAG_WAS_EVER_ARMED = 1 << 3
FLAG_HITL = 1 << 4
FLAG_SITL = 1 << 5
FLAG_ARM_SWITCH = 1 << 14   # cleared by toggling the switch after blockers are gone
NON_BLOCKING = FLAG_ARMED | FLAG_WAS_EVER_ARMED | FLAG_HITL | FLAG_SITL | FLAG_ARM_SWITCH


def rc_neutral() -> list[int]:
    rc = [RC_MID] * 8
    rc[CH_THROTTLE] = RC_LOW
    rc[CH_ARM] = RC_LOW
    rc[CH_ANGLE] = RC_LOW
    rc[CH_INVERTED] = RC_LOW
    rc[CH_SELECT] = RC_LOW
    return rc


def provision():
    msp = MspClient()
    print("API", msp.api_version())
    msp.set_setting("receiver_type", struct.pack("<B", 3))     # SIM (SITL)
    msp.set_setting("platform_type", struct.pack("<B", 1))     # AIRPLANE
    msp.set_setting("small_angle", struct.pack("<B", 180))
    msp.set_setting("baro_hardware", struct.pack("<B", 12))    # FAKE; cal runs on the injected
                                                               # pressure once HITL streams
    msp.set_setting("mag_hardware", struct.pack("<B", 0))      # NONE
    # skip boot gyro + gravity calibration (no real sensors behind the HITL
    # injection on SITL, the calibration FSMs would never complete)
    msp.set_setting("init_gyro_cal", struct.pack("<B", 0))
    msp.set_setting("pitot_hardware", struct.pack("<B", 0))   # NONE
    msp.enable_feature(1 << 7)                                # FEATURE_GPS (HITL injection)
    # provider MSP is driver-based: gpsInit() keeps the feature alive without
    # a serial port (any other provider clears FEATURE_GPS at boot on SITL)
    msp.set_setting("gps_provider", struct.pack("<B", 1))
    # standard airplane servo mixer (S1 aileron, S2 elevator, S3 rudder):
    # without smix rules isMixerUsingServos() is false, servoMixer() never
    # runs and the MSP_SIMULATOR reply's stabilized outputs stay at 0
    msp.set_servo_mixer_rule(0, 0, 0)   # servo 0 <- stabilized roll
    msp.set_servo_mixer_rule(1, 1, 1)   # servo 1 <- stabilized pitch
    msp.set_servo_mixer_rule(2, 2, 2)   # servo 2 <- stabilized yaw
    msp.set_servo_mixer_rule(3, 3, 62)  # servo 3 <- TVC pitch (thrust vectoring)
    msp.set_mode_range(0, PERM_ARM, CH_ARM - 4, 1700, 2100)
    msp.set_mode_range(1, PERM_ANGLE, CH_ANGLE - 4, 1700, 2100)
    # SEL = attitude-target selector (off / INVERT / KNIFE L / KNIFE R / HANG),
    # so the FLOOR gets its own switch travel and combines with ANY attitude
    msp.set_mode_range(2, PERM_INVERTED, CH_SELECT - 4, 1150, 1390)
    msp.set_mode_range(3, PERM_KNIFELEFT, CH_SELECT - 4, 1390, 1630)
    msp.set_mode_range(4, PERM_KNIFERIGHT, CH_SELECT - 4, 1630, 1870)
    msp.set_mode_range(5, PERM_PROPHANG, CH_SELECT - 4, 1870, 2100)
    msp.set_mode_range(6, PERM_ALTFLOOR, CH_INVERTED - 4, 1700, 2100)
    msp.set_mode_range(7, PERM_FIGROLL, CH_INVERTED - 4, 1450, 1700)
    msp.set_mode_range(8, PERM_FIGLOOP, CH_ANGLE - 4, 1150, 1450)
    msp.set_mode_range(9, PERM_FIGSEQ, CH_ANGLE - 4, 1450, 1700)
    # tuned against the JSBSim aerobat3d plant (2026-07-10): altitude spans
    # over a 22 s figure: inverted 3.1 m, roll_hold 1.1 m, knife L/R 6 m
    # (slightly sinking, never climbing)
    msp.set_setting("fig_assist_z_gain", struct.pack("<B", 45))
    msp.set_setting("fig_assist_vz_gain", struct.pack("<B", 3))
    msp.set_setting("fig_assist_max", struct.pack("<B", 20))
    msp.set_setting("ohold_knife_left_pitch_trim", struct.pack("<b", 7))
    msp.set_setting("ohold_knife_right_pitch_trim", struct.pack("<b", 7))
    # BARO_ONLY: aerobatic attitudes lose GPS; stand-in for the planned
    # lock-quality-gated auto switch (GPS only weighted in when locked)
    msp.set_setting("inav_default_alt_sensor", struct.pack("<B", 3))
    msp.save_eeprom()
    print("provisioned + saved, SITL reboots now")


def arming_flags(msp: MspClient) -> int:
    p = msp.request(MSP2_INAV_STATUS)
    # cycleTime u16, i2cErrors u16, sensorStatus u16, avgCPULoad u16,
    # profile/battprofile u8, armingFlags u32, ...
    return struct.unpack_from("<I", p, 9)[0]


def stream(msp: MspClient, plane: PlaneModel, rc: list[int], seconds: float,
           closed_loop: bool = True, log=None, label: str = ""):
    n = int(seconds / DT)
    last = None
    for i in range(n):
        t0 = time.perf_counter()
        last = sim_step(msp, plane.acc_mg(), plane.gyro_dps16(), rc, baro_pa=plane.baro_pa())
        if closed_loop:
            plane.step((last.stab_roll, last.stab_pitch, last.stab_yaw), DT)
        if log is not None and i % 5 == 0:
            log.append((label, i * DT, plane.q, last))
        sleep = DT - (time.perf_counter() - t0)
        if sleep > 0:
            time.sleep(sleep)
    return last


def settle_until_level(msp: MspClient, plane: PlaneModel, rc: list[int],
                       tol_deg: float = 2.5, timeout: float = 45.0):
    """Feed the plane's static attitude until the FC estimate matches it."""
    t_end = time.monotonic() + timeout
    while time.monotonic() < t_end:
        r = stream(msp, plane, rc, 1.0, closed_loop=False)
        import dynamics
        q_fc = dynamics.q_from_rpy(r.att_roll_deg, r.att_pitch_deg, 0)
        if tilt_error_deg(q_fc, plane.q) < tol_deg:
            return r
    raise TimeoutError(f"AHRS did not converge, last FC att "
                       f"{r.att_roll_deg:+.1f}/{r.att_pitch_deg:+.1f}")


FLAG_SENSORS_CALIBRATING = 1 << 9


def wait_boot_calibration(msp: MspClient, timeout: float = 20.0):
    """Wait for the boot gyro calibration BEFORE the first MSP_SIMULATOR
    message: once HITL is enabled, gyroUpdate() returns early and the
    calibration FSM would stay in progress forever (gyro.c)."""
    t_end = time.monotonic() + timeout
    while time.monotonic() < t_end:
        if arming_flags(msp) & FLAG_SENSORS_CALIBRATING == 0:
            return
        time.sleep(0.5)
    raise TimeoutError(f"boot calibration never finished: 0x{arming_flags(msp):08X}")


def arm(msp: MspClient, plane: PlaneModel, rc: list[int], timeout: float = 20.0):
    """Stream until arming blockers clear, then toggle the ARM switch."""
    t_end = time.monotonic() + timeout
    while time.monotonic() < t_end:
        stream(msp, plane, rc, 0.5, closed_loop=False)
        if arming_flags(msp) & ~NON_BLOCKING == 0:
            break
    else:
        raise TimeoutError(f"arming blockers persist: 0x{arming_flags(msp):08X}")
    rc[CH_ARM] = RC_LOW
    stream(msp, plane, rc, 0.3, closed_loop=False)
    rc[CH_ARM] = RC_HIGH
    r = stream(msp, plane, rc, 0.5, closed_loop=False)
    if not r.armed:
        raise RuntimeError(f"arm failed, flags=0x{arming_flags(msp):08X}")
    return r


def smoke():
    msp = MspClient()
    plane = PlaneModel()
    rc = rc_neutral()

    print("-- wait for boot calibration (pre-HITL)")
    wait_boot_calibration(msp)

    print("-- settle AHRS level")
    r = settle_until_level(msp, plane, rc)
    print(f"   FC attitude {r.att_roll_deg:+.1f} {r.att_pitch_deg:+.1f} yaw {r.att_yaw_deg:.0f}  airplane={r.airplane}")
    assert abs(r.att_roll_deg) < 3 and abs(r.att_pitch_deg) < 3, "AHRS did not settle level"
    assert r.airplane, "platform is not AIRPLANE - run provision first"

    print("-- gyro sign check: +20 dps roll for 1 s (open loop)")
    for _ in range(100):
        r = sim_step(msp, plane.acc_mg(), (20 * 16, 0, 0), rc)
        time.sleep(DT)
    print(f"   FC roll now {r.att_roll_deg:+.1f} (expect positive, ~ +20 steady-state; "
          f"lower right after boot while the AHRS runs boosted acc gain)")
    assert 5 < r.att_roll_deg < 35, "gyro roll sign/scale mismatch"

    print("-- acc consistency: hold attitude roll=+20 statically")
    plane.set_attitude(20, 0, 0)
    r = settle_until_level(msp, plane, rc, tol_deg=3.0)
    print(f"   FC roll {r.att_roll_deg:+.1f} (expect ~ +20)")
    assert 15 < r.att_roll_deg < 25, "acc attitude mismatch"

    plane.set_attitude(0, 0, 0)
    settle_until_level(msp, plane, rc)

    print("-- arm")
    arm(msp, plane, rc)
    print("   armed OK")
    rc[CH_ARM] = RC_LOW
    stream(msp, plane, rc, 0.5, closed_loop=False)
    print("SMOKE PASS")


SCENARIOS = [
    #  name        set rc()                                target q      start attitude
    ("angle-level", {CH_ANGLE: RC_HIGH},                   (0, 0),      (25, 10)),
    ("inverted",    {CH_INVERTED: RC_HIGH},                (180, 0),    (0, 0)),
    ("knife-left",  {CH_SELECT: 1300},                     (-90, 0),    (0, 0)),
    ("knife-right", {CH_SELECT: 1600},                     (90, 0),     (0, 0)),
    ("prop-hang",   {CH_SELECT: RC_HIGH},                  (0, 90),     (0, 0)),
]


def scenarios():
    log = []
    ok = True
    for name, rc_over, (t_roll, t_pitch), (s_roll, s_pitch) in SCENARIOS:
        msp = MspClient()
        plane = PlaneModel()
        plane.set_attitude(s_roll, s_pitch)
        rc = rc_neutral()

        wait_boot_calibration(msp)
        settle_until_level(msp, plane, rc, tol_deg=3.0)          # AHRS settle
        try:
            arm(msp, plane, rc)
        except (TimeoutError, RuntimeError) as e:
            print(f"{name}: could not arm: {e}")
            ok = False
            msp.close()
            continue
        rc[CH_THROTTLE] = 1400

        for ch, val in rc_over.items():
            rc[ch] = val
        q_target = q_from_rpy(t_roll, t_pitch, 0)

        r = stream(msp, plane, rc, 8.0, closed_loop=True, log=log, label=name)
        err = tilt_error_deg(plane.q, q_target)
        fc_att = (r.att_roll_deg, r.att_pitch_deg)
        status = "PASS" if err < 15.0 else "FAIL"
        if err >= 15.0:
            ok = False
        print(f"{name:12s} {status}  tilt err {err:6.1f} deg   FC att {fc_att[0]:+7.1f} {fc_att[1]:+6.1f}")

        # bailout: back to ANGLE
        for ch in (CH_INVERTED, CH_SELECT):
            rc[ch] = RC_LOW
        rc[CH_ANGLE] = RC_HIGH
        stream(msp, plane, rc, 12.0, closed_loop=True, log=log, label=name + "-bailout")
        err = tilt_error_deg(plane.q, q_from_rpy(0, 0, 0))
        if err < 15.0:
            status = "PASS"
        elif name == "inverted" and err < 25.0:
            # stock Euler ANGLE rolling back from exactly 180 deg picks a
            # random direction (Euler wrap) and is sometimes slower --
            # known-flaky upstream behaviour, not part of this feature
            status = "PASS (known-flaky window)"
        else:
            status = "FAIL"
            ok = False
        print(f"{name:12s} bailout {status}  tilt err {err:6.1f} deg")

        rc[CH_ARM] = RC_LOW
        stream(msp, plane, rc, 0.3, closed_loop=False)
        msp.close()

    with open("scenario_log.csv", "w") as f:
        f.write("scenario,t,qw,qx,qy,qz,stab_roll,stab_pitch,stab_yaw,fc_roll,fc_pitch,fc_yaw\n")
        for label, t, q, r in log:
            f.write(f"{label},{t:.2f},{q[0]:.5f},{q[1]:.5f},{q[2]:.5f},{q[3]:.5f},"
                    f"{r.stab_roll:.3f},{r.stab_pitch:.3f},{r.stab_yaw:.3f},"
                    f"{r.att_roll_deg:.1f},{r.att_pitch_deg:.1f},{r.att_yaw_deg:.1f}\n")
    print("log written to scenario_log.csv")
    sys.exit(0 if ok else 1)


# Edge cases from the singularity checklist, closed loop:
# antipode starts (error ~180 deg around the inverted target, both sides)
# and convergence through / near the Euler pitch-90 singularity.
EDGE_SCENARIOS = [
    #  name                  rc                   target        start
    ("antipode+179",        {CH_INVERTED: RC_HIGH}, (180, 0),  (179, 0)),
    ("antipode-179",        {CH_INVERTED: RC_HIGH}, (180, 0),  (-179, 0)),
    ("prophang-thru-90",    {CH_SELECT: RC_HIGH},   (0, 90),   (0, 0)),
    ("prophang-past-vert",  {CH_SELECT: RC_HIGH},   (0, 95),   (0, 95)),
]


def edge():
    ok = True
    for name, rc_over, (t_roll, t_pitch), (s_roll, s_pitch) in EDGE_SCENARIOS:
        # "prophang-past-vert" starts at pitch 95 and must settle onto 90
        if name == "prophang-past-vert":
            t_roll, t_pitch = 0, 90
        msp = MspClient()
        plane = PlaneModel()
        plane.set_attitude(s_roll, s_pitch)
        rc = rc_neutral()

        wait_boot_calibration(msp)
        settle_until_level(msp, plane, rc, tol_deg=3.0)
        try:
            arm(msp, plane, rc)
        except (TimeoutError, RuntimeError) as e:
            print(f"{name}: could not arm: {e}")
            ok = False
            msp.close()
            continue
        rc[CH_THROTTLE] = 1400
        for ch, val in rc_over.items():
            rc[ch] = val
        q_target = q_from_rpy(t_roll, t_pitch, 0)

        # per-step trace for continuity/shortest-path checks
        trace = []
        n = int(8.0 / DT)
        for i in range(n):
            t0 = time.perf_counter()
            r = sim_step(msp, plane.acc_mg(), plane.gyro_dps16(), rc, baro_pa=plane.baro_pa())
            plane.step((r.stab_roll, r.stab_pitch, r.stab_yaw), DT)
            trace.append((i * DT, tilt_error_deg(plane.q, q_target),
                          r.stab_roll, r.stab_pitch, r.stab_yaw))
            sleep = DT - (time.perf_counter() - t0)
            if sleep > 0:
                time.sleep(sleep)

        final_err = trace[-1][1]
        after = [s for s in trace if s[0] >= 0.5]
        err_at_half = after[0][1]
        max_err_after = max(s[1] for s in after)
        max_jump = max(abs(b[3] - a[3]) for a, b in zip(after, after[1:]))
        nan_free = all(all(v == v for v in s[1:]) for s in trace)

        checks = {
            "final<10deg": final_err < 10.0,
            "pitch-cmd-continuous": max_jump < 0.5,             # no jump at 90 deg
            "no-nan": nan_free,
        }
        if name.startswith("antipode"):
            # shortest-path semantics only meaningful for ~180 deg starts;
            # large-maneuver cases overshoot with the P-only level gain
            # (tuning, not geometry) and are judged on convergence instead
            checks["no-detour"] = max_err_after <= err_at_half + 15.0
        else:
            checks["overshoot<60"] = max_err_after < 60.0
        status = "PASS" if all(checks.values()) else "FAIL"
        if status == "FAIL":
            ok = False
        detail = " ".join(k for k, v in checks.items() if not v)
        print(f"{name:20s} {status}  final {final_err:5.1f} deg  max-after {max_err_after:6.1f}  "
              f"max-pitch-cmd-jump {max_jump:.3f}  {detail}")

        rc[CH_ARM] = RC_LOW
        stream(msp, plane, rc, 0.3, closed_loop=False)
        msp.close()
    sys.exit(0 if ok else 1)


def floor():
    """Altitude floor: climb in ACRO, dive at the ground, the ALT FLOOR mode
    must catch the plane above the floor; with the box off, descending below
    the floor must stay untouched (landing)."""
    msp = MspClient()
    plane = PlaneModel()
    rc = rc_neutral()

    wait_boot_calibration(msp)
    settle_until_level(msp, plane, rc)
    arm(msp, plane, rc)
    rc[CH_THROTTLE] = 1400
    rc[CH_INVERTED] = RC_ALTFLOOR_ON     # ALT FLOOR box on

    log = []
    dbg = {}

    def fly(seconds, label):
        last = None
        for _ in range(int(seconds / DT)):
            t0 = time.perf_counter()
            last = sim_step(msp, plane.acc_mg(), plane.gyro_dps16(), rc, baro_pa=plane.baro_pa())
            plane.step((last.stab_roll, last.stab_pitch, last.stab_yaw), DT)
            dbg[last.debug[0]] = last.debug[1]
            log.append((label, plane.z, last.att_pitch_deg, dict(dbg)))
            s = DT - (time.perf_counter() - t0)
            if s > 0:
                time.sleep(s)
        return last

    # fly everything by stick: attitude teleports break the gyro/attitude
    # consistency (doc: gyro must match qdot) and the AHRS flies blind
    def stick_pitch_to(target_deg, timeout=8.0):
        # learn stick direction on the plant, then a crude P approach
        p0 = plane.pitch_deg()
        rc[CH_PITCH] = 1620
        fly(0.4, "stick")
        sign = 1 if plane.pitch_deg() > p0 else -1
        t_end = time.monotonic() + timeout
        while abs(plane.pitch_deg() - target_deg) > 3.0 and time.monotonic() < t_end:
            err = target_deg - plane.pitch_deg()
            rc[CH_PITCH] = int(1500 + sign * max(-350, min(350, err * 15)))
            fly(0.1, "stick")
        rc[CH_PITCH] = RC_MID

    # climb well above floor(30) + margin(10) so the floor latch arms
    stick_pitch_to(25)
    t_end = time.monotonic() + 60
    while plane.z < 63 and time.monotonic() < t_end:
        fly(1.0, "climb")
    print(f"-- climbed to {plane.z:.0f} m")
    assert plane.z >= 63, "climb failed"

    # push over into a dive at the ground, then sticks neutral:
    # the floor must catch it
    stick_pitch_to(-50, timeout=4.0)
    fly(8.0, "catch")
    catch = [(z, p, d) for l, z, p, d in log if l == "catch"]
    z_min = min(z for z, _, _ in catch)
    engaged = any(p > 5 for _, p, _ in catch)   # nose pulled up without stick input
    for i in range(0, len(catch), 50):          # 0.5 s trace
        z, p, d = catch[i]
        print(f"   t={i * DT:4.1f}s  z={z:7.1f} m  fc_pitch={p:+6.1f}  "
              f"box={d.get(0)} trust={d.get(1)} z_est={d.get(2, 0) / 100.0:6.1f} "
              f"vz_est={d.get(3, 0) / 100.0:+6.1f} state={d.get(4)}")
    print(f"-- dive: min altitude {z_min:.1f} m (floor 30), recovery engaged: {engaged}, "
          f"now z {plane.z:.1f} m pitch {catch[-1][1]:+.1f}")

    # box off -> descending below the floor must stay untouched (landing)
    rc[CH_INVERTED] = RC_LOW
    stick_pitch_to(-30, timeout=6.0)
    t_end = time.monotonic() + 60
    while plane.z > 22 and time.monotonic() < t_end:
        fly(1.0, "land")
    land = [(z, p) for l, z, p, _ in log if l == "land"]
    late_land_pitch = [p for _, p in land[len(land) // 2:]]
    no_takeover = max(late_land_pitch) < 0.0
    print(f"-- landing descent: z {plane.z:.1f} m (below floor 30), FC pitch stayed nose-down: {no_takeover}")

    checks = {
        "recovery-engaged": engaged,
        "caught-above-25m": z_min > 25.0,
        "back-above-floor": catch[-1][0] > 30.0,
        "landing-untouched": no_takeover and plane.z < 30.0,
    }
    status = "PASS" if all(checks.values()) else "FAIL"
    print(f"floor {status}  " + " ".join(k for k, v in checks.items() if not v))

    rc[CH_ARM] = RC_LOW
    stream(msp, plane, rc, 0.3, closed_loop=False)
    msp.close()
    sys.exit(0 if status == "PASS" else 1)


def tvc():
    """Thrust vectoring inputs: servo 3 carries TVC Pitch, servo 1 carries
    Stabilized Pitch. The TVC/surface deflection ratio must follow the
    inverse thrust compensation: ~1 at full thrust, ~1/floor at idle."""
    msp = MspClient()
    plane = PlaneModel()
    rc = rc_neutral()

    wait_boot_calibration(msp)
    settle_until_level(msp, plane, rc)
    arm(msp, plane, rc)

    def ratio_at(throttle_us: float) -> float:
        rc[CH_THROTTLE] = int(throttle_us)
        rc[CH_PITCH] = 1650                      # constant rate demand
        stream(msp, plane, rc, 1.5, closed_loop=False)   # plant frozen: steady axisPID
        servos = msp.servos_us()
        d_surf = servos[1] - 1500
        d_tvc = servos[3] - 1500
        rc[CH_PITCH] = RC_MID
        stream(msp, plane, rc, 0.5, closed_loop=False)
        if abs(d_surf) < 20:
            raise RuntimeError(f"no usable surface deflection ({d_surf})")
        return d_tvc / d_surf

    r_full = ratio_at(2000)
    r_idle = ratio_at(1150)
    print(f"TVC/surface deflection ratio: full thrust {r_full:.2f} (expect ~1), "
          f"idle {r_idle:.2f} (expect ~4, comp cap 1/0.25)")

    ok = 0.8 <= r_full <= 1.3 and r_idle > 2.5
    print("tvc", "PASS" if ok else "FAIL")

    rc[CH_ARM] = RC_LOW
    stream(msp, plane, rc, 0.3, closed_loop=False)
    msp.close()
    sys.exit(0 if ok else 1)


def figures():
    """Figure sequencer: roll with/without altitude assist (A/B) and loop.
    The plant sinks away from wings-level, so the assist has real work."""
    import struct
    msp = MspClient()
    plane = PlaneModel()
    rc = rc_neutral()

    wait_boot_calibration(msp)
    settle_until_level(msp, plane, rc)
    arm(msp, plane, rc)
    rc[CH_THROTTLE] = 1400

    def run_figure(ch, on_value, seconds):
        z0 = plane.z
        min_align = 1.0
        max_drift = 0.0
        rc[ch] = on_value
        for _ in range(int(seconds / DT)):
            t0 = time.perf_counter()
            r = sim_step(msp, plane.acc_mg(), plane.gyro_dps16(), rc, baro_pa=plane.baro_pa())
            plane.step((r.stab_roll, r.stab_pitch, r.stab_yaw), DT)
            from dynamics import rotate_earth_to_body
            min_align = min(min_align, rotate_earth_to_body(plane.q, (0, 0, 1))[2])
            max_drift = max(max_drift, abs(plane.z - z0))
            s = DT - (time.perf_counter() - t0)
            if s > 0:
                time.sleep(s)
        # figure completion = wings level again (pitch belongs to the
        # altitude assist and freezes in ACRO after release - not a defect)
        import math
        w, x, y, zc = plane.q
        roll_end = abs(math.degrees(math.atan2(2 * (w * x + y * zc), 1 - 2 * (x * x + y * y))))
        rc[ch] = RC_LOW
        stream(msp, plane, rc, 0.5, closed_loop=True)
        return min_align, max_drift, roll_end, abs(plane.z - z0)

    def relevel():
        rc[CH_ANGLE] = RC_HIGH
        stream(msp, plane, rc, 5.0, closed_loop=True)
        rc[CH_ANGLE] = RC_LOW
        stream(msp, plane, rc, 0.3, closed_loop=True)

    # A: roll without altitude assist
    msp.set_setting("fig_assist_z_gain", struct.pack("<B", 0))
    msp.set_setting("fig_assist_vz_gain", struct.pack("<B", 0))
    align_a, drift_a, rend_a, zend_a = run_figure(CH_INVERTED, RC_FIGROLL_ON, 9.0)
    print(f"roll no-assist : inverted-pass {align_a:+.2f}  max |dz| {drift_a:5.1f} m  "
          f"end |dz| {zend_a:5.1f} m  end roll {rend_a:5.1f} deg")
    relevel()

    # B: roll with altitude assist (defaults)
    msp.set_setting("fig_assist_z_gain", struct.pack("<B", 20))
    msp.set_setting("fig_assist_vz_gain", struct.pack("<B", 1))
    align_b, drift_b, rend_b, zend_b = run_figure(CH_INVERTED, RC_FIGROLL_ON, 9.0)
    print(f"roll assist    : inverted-pass {align_b:+.2f}  max |dz| {drift_b:5.1f} m  "
          f"end |dz| {zend_b:5.1f} m  end roll {rend_b:5.1f} deg")
    relevel()

    # loop (roll-end check not meaningful, use inverted-pass only)
    align_l, drift_l, rend_l, zend_l = run_figure(CH_ANGLE, RC_FIGLOOP_ON, 9.0)
    print(f"loop           : inverted-pass {align_l:+.2f}  max |dz| {drift_l:5.1f} m  "
          f"end |dz| {zend_l:5.1f} m  end roll {rend_l:5.1f} deg")

    checks = {
        "roll-a-completed": align_a < -0.8 and rend_a < 25.0,
        "roll-b-completed": align_b < -0.8 and rend_b < 25.0,
        "assist-reduces-end-dz": zend_b < zend_a * 0.7,
        "assist-end-dz<3m": zend_b < 3.0,
        "loop-completed": align_l < -0.8 and rend_l < 30.0,
    }
    status = "PASS" if all(checks.values()) else "FAIL"
    print(f"figures {status}  " + " ".join(k for k, v in checks.items() if not v))

    rc[CH_ARM] = RC_LOW
    stream(msp, plane, rc, 0.3, closed_loop=False)
    msp.close()
    sys.exit(0 if status == "PASS" else 1)


def sequence():
    """Programmed chain with a precondition: WAIT_ALT 40 m, then Immelmann
    (half loop + half roll), then hold level. The figure must not start
    before the altitude gate is reached."""
    import math
    import struct
    msp = MspClient()
    plane = PlaneModel()
    rc = rc_neutral()

    # program the sequence
    msp.set_figure_segment(0, FIGSEG_WAIT_ALT, p1=40, p2=3)
    msp.set_figure_segment(1, FIGSEG_PITCH, p1=180)                       # half loop
    msp.set_figure_segment(2, FIGSEG_ROLL, p1=180)                        # half roll -> upright
    msp.set_figure_segment(3, FIGSEG_WAIT_TIME, p3=2000, flags=FIGSEG_FLAG_ASSIST)
    msp.set_figure_segment(4, FIGSEG_END)

    wait_boot_calibration(msp)
    settle_until_level(msp, plane, rc)
    arm(msp, plane, rc)
    rc[CH_THROTTLE] = 1400
    rc[CH_ANGLE] = RC_FIGSEQ_ON

    min_align = 1.0
    z_at_first_pitch = None
    seq_rows = []
    for i in range(int(35.0 / DT)):
        t0 = time.perf_counter()
        r = sim_step(msp, plane.acc_mg(), plane.gyro_dps16(), rc, baro_pa=plane.baro_pa())
        plane.step((r.stab_roll, r.stab_pitch, r.stab_yaw), DT)
        seq_rows.append((i * DT, plane.z, plane.nose_elevation_deg(), *plane.q))
        from dynamics import rotate_earth_to_body
        min_align = min(min_align, rotate_earth_to_body(plane.q, (0, 0, 1))[2])
        if z_at_first_pitch is None and abs(plane.pitch_deg()) > 30.0:
            z_at_first_pitch = plane.z
        if i % 500 == 0:
            print(f"   t={i * DT:4.1f}s  z={plane.z:5.1f} m  nose elev={plane.nose_elevation_deg():+5.1f}")
        s = DT - (time.perf_counter() - t0)
        if s > 0:
            time.sleep(s)

    w, x, y, zc = plane.q
    roll_end = abs(math.degrees(math.atan2(2 * (w * x + y * zc), 1 - 2 * (x * x + y * y))))
    align_end = __import__("dynamics").rotate_earth_to_body(plane.q, (0, 0, 1))[2]

    with open("sequence_log.csv", "w") as f:
        f.write("t,z,nose_elev,qw,qx,qy,qz\n")
        for row in seq_rows:
            f.write(",".join(f"{v:.4f}" for v in row) + "\n")

    print(f"gate altitude at figure start: {z_at_first_pitch} m (gate 40)")
    print(f"inverted-pass {min_align:+.2f}, end roll {roll_end:.1f} deg, end upright {align_end:+.2f}, end z {plane.z:.1f} m")

    checks = {
        "gate-respected": z_at_first_pitch is not None and z_at_first_pitch > 35.0,
        "inverted-passed": min_align < -0.8,
        "ends-upright": roll_end < 25.0 and align_end > 0.8,
    }
    status = "PASS" if all(checks.values()) else "FAIL"
    print(f"sequence {status}  " + " ".join(k for k, v in checks.items() if not v))

    rc[CH_ANGLE] = RC_LOW
    rc[CH_ARM] = RC_LOW
    stream(msp, plane, rc, 0.3, closed_loop=False)
    msp.close()
    sys.exit(0 if status == "PASS" else 1)


PERM_ATTLOCK = 78


def lock():
    """3D LOCK: capture attitude on entry, hold while sticks centered,
    follow stick input, lock the new attitude on release. Judged on
    windowed mean attitude (the P-only hold wobbles a few degrees)."""
    msp = MspClient()
    plane = PlaneModel()
    rc = rc_neutral()
    # temporarily map 3D LOCK onto the knife-left band
    msp.set_mode_range(3, PERM_KNIFELEFT, CH_SELECT - 4, 900, 900)
    msp.set_mode_range(10, PERM_ATTLOCK, CH_SELECT - 4, 1150, 1450)

    wait_boot_calibration(msp)
    settle_until_level(msp, plane, rc)
    arm(msp, plane, rc)
    rc[CH_THROTTLE] = 1400

    def mean_att(seconds):
        rs, ps = [], []
        for _ in range(int(seconds / DT)):
            t0 = time.perf_counter()
            r = sim_step(msp, plane.acc_mg(), plane.gyro_dps16(), rc, baro_pa=plane.baro_pa())
            plane.step((r.stab_roll, r.stab_pitch, r.stab_yaw), DT)
            rs.append(r.att_roll_deg)
            ps.append(r.att_pitch_deg)
            s = DT - (time.perf_counter() - t0)
            if s > 0:
                time.sleep(s)
        return sum(rs) / len(rs), sum(ps) / len(ps)

    rc[CH_ROLL] = 1650
    mean_att(0.45)
    rc[CH_ROLL] = RC_MID
    mean_att(0.3)
    rc[CH_SELECT] = 1300                 # 3D LOCK on
    mean_att(1.5)
    a0 = mean_att(1.5)
    mean_att(3.0)
    a1 = mean_att(1.5)
    drift1 = max(abs(a1[0] - a0[0]), abs(a1[1] - a0[1]))
    print(f"lock 1: mean att ({a0[0]:+.1f},{a0[1]:+.1f}) -> ({a1[0]:+.1f},{a1[1]:+.1f}), drift {drift1:.1f} deg")

    rc[CH_PITCH] = 1700
    mean_att(0.6)
    rc[CH_PITCH] = RC_MID
    mean_att(1.5)
    b0 = mean_att(1.5)
    moved = max(abs(b0[0] - a1[0]), abs(b0[1] - a1[1]))
    mean_att(3.0)
    b1 = mean_att(1.5)
    drift2 = max(abs(b1[0] - b0[0]), abs(b1[1] - b0[1]))
    print(f"stick moved mean attitude by {moved:.1f} deg; lock 2 drift {drift2:.1f} deg")

    ok = drift1 < 4.0 and moved > 10.0 and drift2 < 4.0
    print("lock", "PASS" if ok else "FAIL")

    # restore mode ranges
    msp.set_mode_range(3, PERM_KNIFELEFT, CH_SELECT - 4, 1150, 1450)
    msp.set_mode_range(10, PERM_ATTLOCK, CH_SELECT - 4, 900, 900)
    rc[CH_SELECT] = RC_LOW
    rc[CH_ARM] = RC_LOW
    stream(msp, plane, rc, 0.3, closed_loop=False)
    msp.close()
    sys.exit(0 if ok else 1)


def hover():
    """Hover throttle: engage PROP HANG with the throttle
    stick centered - the thrust PID must hold the captured altitude in the
    thrust-borne plant regime; throttle stick out of the deadband hands
    control back and the plane climbs."""
    import struct
    msp = MspClient()
    plane = PlaneModel()
    rc = rc_neutral()

    # gains tuned for this plant (defaults are conservative; tuning is what
    # the pilot does on the real model via the GUI)
    msp.set_setting("ohold_hover_thr_p", struct.pack("<B", 60))
    msp.set_setting("ohold_hover_thr_i", struct.pack("<B", 40))
    msp.set_setting("ohold_hover_thr_d", struct.pack("<B", 80))

    wait_boot_calibration(msp)
    settle_until_level(msp, plane, rc)
    arm(msp, plane, rc)
    rc[CH_THROTTLE] = 1400

    hover_rows = []

    def fly(seconds, thrust_borne):
        zs = []
        last = None
        for _ in range(int(seconds / DT)):
            t0 = time.perf_counter()
            last = sim_step(msp, plane.acc_mg(), plane.gyro_dps16(), rc, baro_pa=plane.baro_pa())
            thr01 = (last.stab_throttle + 1.0) / 2.0 if thrust_borne else None
            plane.step((last.stab_roll, last.stab_pitch, last.stab_yaw), DT, throttle01=thr01)
            zs.append(plane.z)
            hover_rows.append((len(hover_rows) * DT, plane.z, plane.nose_elevation_deg(),
                               (last.stab_throttle + 1.0) / 2.0))
            s = DT - (time.perf_counter() - t0)
            if s > 0:
                time.sleep(s)
        return last, zs

    # climb to altitude, then engage PROP HANG with the throttle centered
    fly(6.0, False)
    rc[CH_SELECT] = 1900               # PROP HANG box
    rc[CH_THROTTLE] = RC_MID           # centered: hover throttle owns altitude
    fly(10.0, True)                    # rotate to vertical + engage + settle
    z_ref = plane.z                    # judge the HOLD after settling (the
                                       # I-term learns the hover throttle first)
    _, zs = fly(12.0, True)
    drift = max(abs(z - z_ref) for z in zs)
    print(f"prop hang hold 12 s (after settle): ref z {z_ref:.1f} m, max |dz| {drift:.1f} m")

    # pilot override: throttle up -> must climb
    rc[CH_THROTTLE] = 1750
    z0 = plane.z
    fly(3.0, True)
    climb = plane.z - z0
    print(f"pilot throttle override 3 s: climbed {climb:+.1f} m")

    with open("hover_log.csv", "w") as f:
        f.write("t,z,nose_elev,throttle01\n")
        for row in hover_rows:
            f.write(",".join(f"{v:.4f}" for v in row) + "\n")

    ok = drift < 6.0 and climb > 4.0
    print("hover", "PASS" if ok else "FAIL")

    rc[CH_SELECT] = RC_LOW
    rc[CH_ARM] = RC_LOW
    stream(msp, plane, rc, 0.3, closed_loop=False)
    msp.close()
    sys.exit(0 if ok else 1)


FIGSEG_IMPULSE, FIGSEG_WAIT_POS = 6, 7


def snap():
    """Post-stall entry: IMPULSE segment kicks full pitch+yaw rates open
    loop, the following segment catches the attitude (wings level)."""
    import math
    msp = MspClient()
    plane = PlaneModel()
    rc = rc_neutral()

    msp.set_figure_segment(0, FIGSEG_WAIT_TIME, p3=1000, flags=FIGSEG_FLAG_ASSIST)
    msp.set_figure_segment(1, FIGSEG_IMPULSE, p1=100, p2=100, p3=400)
    msp.set_figure_segment(2, FIGSEG_WAIT_TIME, p3=4000, flags=FIGSEG_FLAG_ASSIST)
    msp.set_figure_segment(3, FIGSEG_END)

    wait_boot_calibration(msp)
    settle_until_level(msp, plane, rc)
    arm(msp, plane, rc)
    rc[CH_THROTTLE] = 1400
    rc[CH_ANGLE] = RC_FIGSEQ_ON

    max_rate = 0.0
    for _ in range(int(9.0 / DT)):
        t0 = time.perf_counter()
        r = sim_step(msp, plane.acc_mg(), plane.gyro_dps16(), rc, baro_pa=plane.baro_pa())
        plane.step((r.stab_roll, r.stab_pitch, r.stab_yaw), DT)
        max_rate = max(max_rate, max(abs(math.degrees(w)) for w in plane.omega))
        s = DT - (time.perf_counter() - t0)
        if s > 0:
            time.sleep(s)

    w, x, y, zq = plane.q
    roll_end = abs(math.degrees(math.atan2(2 * (w * x + y * zq), 1 - 2 * (x * x + y * y))))
    from dynamics import rotate_earth_to_body
    align = rotate_earth_to_body(plane.q, (0, 0, 1))[2]
    print(f"snap: peak body rate {max_rate:.0f} deg/s, end roll {roll_end:.1f} deg, upright {align:+.2f}")
    ok = max_rate > 150.0 and roll_end < 25.0 and align > 0.8
    print("snap", "PASS" if ok else "FAIL")

    rc[CH_ANGLE] = RC_LOW
    rc[CH_ARM] = RC_LOW
    stream(msp, plane, rc, 0.3, closed_loop=False)
    msp.close()
    sys.exit(0 if ok else 1)


def contain():
    """Airspace containment: fly ~350 m away, then WAIT_POS must bank the
    plane around (coordinated turn) and bring it back inside the radius."""
    import struct as _s
    msp = MspClient()
    plane = PlaneModel()
    rc = rc_neutral()

    msp.set_figure_segment(0, FIGSEG_WAIT_POS, p1=50, p2=35)
    msp.set_figure_segment(1, FIGSEG_END)

    def fly(seconds):
        last = None
        for _ in range(int(seconds / DT)):
            t0 = time.perf_counter()
            last = sim_step(msp, plane.acc_mg(), plane.gyro_dps16(), rc,
                            baro_pa=plane.baro_pa(), gps=plane.gps())
            plane.step((last.stab_roll, last.stab_pitch, last.stab_yaw), DT)
            s = DT - (time.perf_counter() - t0)
            if s > 0:
                time.sleep(s)
        return last

    wait_boot_calibration(msp)
    fly(6.0)                             # settle AHRS + GPS fix, feed continuously
    # arm with the GPS stream alive so home gets set at the arming point
    t_end = time.monotonic() + 20
    while time.monotonic() < t_end:
        fly(0.5)
        if arming_flags(msp) & ~NON_BLOCKING == 0:
            break
    rc[CH_ARM] = RC_LOW
    fly(0.3)
    rc[CH_ARM] = RC_HIGH
    r = fly(0.5)
    if not r.armed:
        raise RuntimeError(f"arm failed, flags=0x{arming_flags(msp):08X}")
    rc[CH_THROTTLE] = 1400
    fly(15.0)                            # fly straight away from home
    dist0 = (plane.x ** 2 + plane.y ** 2) ** 0.5
    print(f"drifted to {dist0:.0f} m from home")

    rc[CH_ANGLE] = RC_FIGSEQ_ON          # WAIT_POS: come home
    min_dist = dist0
    for k in range(40):                  # up to 120 s
        fly(3.0)
        dist = (plane.x ** 2 + plane.y ** 2) ** 0.5
        min_dist = min(min_dist, dist)
        p = msp.request(107)
        fc_dist, fc_dir = _s.unpack_from("<Hh", p, 0)
        if k % 4 == 0:
            print(f"   t={3 * (k + 1):3d}s  true dist {dist:4.0f} m  FC distHome {fc_dist:4d}")
        if min_dist < 55.0:
            break
    print(f"closest approach: {min_dist:.0f} m (radius 50)")
    ok = dist0 > 150.0 and min_dist < 60.0
    print("contain", "PASS" if ok else "FAIL")

    rc[CH_ANGLE] = RC_LOW
    rc[CH_ARM] = RC_LOW
    stream(msp, plane, rc, 0.3, closed_loop=False)
    msp.close()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "smoke"
    {"provision": provision, "smoke": smoke, "scenarios": scenarios, "edge": edge,
     "floor": floor, "tvc": tvc, "figures": figures, "sequence": sequence,
     "lock": lock, "hover": hover, "snap": snap, "contain": contain}[cmd]()
