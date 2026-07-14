"""Multirotor crash-detection functional test in SITL (no JSBSim needed).

Provisions SITL as a quad, arms, 'flies' (throttle up so the in-flight
latch arms), then injects an impact spike followed by stillness straight
through the HITL sensor stream and reads MSP_MOTOR to confirm ALL motors
drop to idle - the copter proof that the cut goes through
getMotorStatus() -> MOTOR_STOPPED and is NOT overridden by the PID mix.

Two-step (a clean reboot between provision and flight is required):
    python mc_crash_test.py --provision   # then restart the SITL container
    python mc_crash_test.py                # runs the crash test

Note on timing: under --lockstep each injected frame is ~1 ms of FC time,
so the fly/still phases are counted in FRAMES (~1000 = 1 s), not seconds -
too few frames and the 1 s in-flight latch / stillness confirm never
elapse in FC time.
"""
import struct
import sys

from msp import MspClient
from hitl import sim_step
from bench import (PlaneModel, rc_neutral, wait_boot_calibration,
                   settle_until_level, arm, CH_THROTTLE, CH_ARM, RC_LOW)

MSP2_COMMON_SET_MOTOR_MIXER = 0x1006
MSP_MOTOR = 104
FLY_FRAMES = 1500      # > 1 s FC time so the in-flight latch arms
STILL_FRAMES = 2500    # > 1 s FC time so the stillness confirm elapses


def mix_raw(v):
    return int((v + 2.0) * 1000)


def set_motor_mixer(msp, idx, thr, roll, pitch, yaw):
    msp.request(MSP2_COMMON_SET_MOTOR_MIXER,
                struct.pack("<BHHHH", idx, mix_raw(thr), mix_raw(roll), mix_raw(pitch), mix_raw(yaw)))


def motors(msp):
    raw = msp.request(MSP_MOTOR)
    n = len(raw) // 2
    return list(struct.unpack("<%dH" % n, raw[:n * 2]))[:4]


def provision_quad():
    msp = MspClient()
    print("API", msp.api_version())
    msp.set_setting("receiver_type", struct.pack("<B", 3))     # SIM
    msp.set_setting("platform_type", struct.pack("<B", 0))     # MULTIROTOR
    msp.set_setting("small_angle", struct.pack("<B", 180))
    msp.set_setting("baro_hardware", struct.pack("<B", 12))    # FAKE
    msp.set_setting("mag_hardware", struct.pack("<B", 0))      # NONE
    msp.set_setting("init_gyro_cal", struct.pack("<B", 0))
    msp.set_setting("pitot_hardware", struct.pack("<B", 0))
    # standard quad X motor mixer (throttle 1 on all; roll/pitch/yaw signs)
    set_motor_mixer(msp, 0, 1.0, -1.0,  1.0, -1.0)   # rear right
    set_motor_mixer(msp, 1, 1.0, -1.0, -1.0,  1.0)   # front right
    set_motor_mixer(msp, 2, 1.0,  1.0,  1.0,  1.0)   # rear left
    set_motor_mixer(msp, 3, 1.0,  1.0, -1.0, -1.0)   # front left
    msp.set_mode_range(0, 0, CH_ARM - 4, 1700, 2100)   # PERM_ARM = 0
    msp.save_eeprom()
    print("quad provisioned + saved; restart the SITL container, then run without --provision")


def main():
    if "--provision" in sys.argv:
        provision_quad()
        return

    msp = MspClient()
    plane = PlaneModel()
    rc = rc_neutral()
    wait_boot_calibration(msp)
    settle_until_level(msp, plane, rc)
    r = arm(msp, plane, rc)
    assert not r.airplane, "platform is not multirotor - provision first"
    print(f"armed={r.armed} airplane={r.airplane} (multirotor)")

    # fly: throttle up long enough for the in-flight latch to arm
    rc[CH_THROTTLE] = 1700
    for _ in range(FLY_FRAMES):
        sim_step(msp, plane.acc_mg(), plane.gyro_dps16(), rc, baro_pa=plane.baro_pa())
    m_fly = motors(msp)
    print(f"flying:            motors={m_fly}")

    # impact: 12 g spike, then stillness (level 1 g, zero rates, frozen baro)
    for _ in range(30):
        sim_step(msp, (0, 0, 12000), (0, 0, 0), rc, baro_pa=plane.baro_pa())
    cut_frame = None
    for k in range(STILL_FRAMES):
        sim_step(msp, (0, 0, 1000), (0, 0, 0), rc, baro_pa=plane.baro_pa())
        if cut_frame is None and max(motors(msp)) < 1150:
            cut_frame = k
    m_cut = motors(msp)
    print(f"after crash:       motors={m_cut}  (cut at still frame {cut_frame})")

    ok = max(m_fly) > 1400 and max(m_cut) < 1150
    print("RESULT:", "PASS - copter motors cut on crash" if ok else "FAIL")

    # gesture: throttle to zero then up -> motor re-allowed
    rc[CH_THROTTLE] = RC_LOW
    for _ in range(1200):
        sim_step(msp, (0, 0, 1000), (0, 0, 0), rc, baro_pa=plane.baro_pa())
    rc[CH_THROTTLE] = 1600
    for _ in range(1500):
        sim_step(msp, (0, 0, 1000), (0, 0, 0), rc, baro_pa=plane.baro_pa())
    m_re = motors(msp)
    print(f"after gesture:     motors={m_re}",
          "(re-allowed)" if max(m_re) > 1400 else "(still cut)")


if __name__ == "__main__":
    main()
