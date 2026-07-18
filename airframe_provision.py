# Copyright (C) 2026 Daniel Haensse
# GPL-3.0-or-later (see repo LICENSE)
"""Per-airframe FC provisioning, driven by airframe_config.py: the bench
core provisioning plus the servo mixer that matches the REAL actuator
set, saved to the FC and copied out as eeprom_<model>.bin. The check
flight ("Einflug") itself lives in the flight: jsbsim_fly.py's `show`
sequence opens with a trim phase that finds the model's level throttle
by feedback and records it in airframe_trim.json.

    python airframe_provision.py <model>      (SITL must be running)
"""
import os
import shutil
import struct
import sys
import time

from airframe_config import AIRFRAMES
from msp import MspClient
import bench

# INAV smix input sources (src/main/flight/servos.h)
IN_ROLL, IN_PITCH, IN_YAW = 0, 1, 2
IN_FLAPS = 14                 # INPUT_FEATURE_FLAPS (FLAPERON mode drives it)
IN_TVC_PITCH, IN_TVC_YAW = 62, 63


def provision_mixer(msp, actuators):
    """Servo mixer for the REAL actuator set. Returns the rule count; the
    caller terminates the list (the FC stops loading at the first rate-0
    rule, so stale tail rules from the core provisioning disappear)."""
    if actuators.startswith("ELEVON"):
        # elevon mix: both surfaces carry roll AND pitch (FC-side mixing,
        # the honest config for xeno/lippisch/arwing/deltastrike/funjet)
        msp.set_servo_mixer_rule(0, 0, IN_ROLL, rate=50)
        msp.set_servo_mixer_rule(1, 0, IN_PITCH, rate=50)
        msp.set_servo_mixer_rule(2, 1, IN_ROLL, rate=-50)
        msp.set_servo_mixer_rule(3, 1, IN_PITCH, rate=50)
        if actuators == "ELEVON_R":
            msp.set_servo_mixer_rule(4, 2, IN_YAW)
            return 5
        if actuators == "ELEVON_TVC":
            msp.set_servo_mixer_rule(4, 2, IN_TVC_PITCH)
            msp.set_servo_mixer_rule(5, 3, IN_TVC_YAW)
            return 6
        return 4
    if actuators == "QH":
        # no rudder on the airframe: NO yaw rule at all - the knife-edge
        # boxes vanish via servoMixerHasYawControl(), the FW's own
        # capability gating
        msp.set_servo_mixer_rule(0, 0, IN_ROLL)
        msp.set_servo_mixer_rule(1, 1, IN_PITCH)
        return 2
    if actuators == "GYRO":
        # rotor tilt is LATERAL only -> it is the roll surface; pitch is
        # the tail elevator, yaw the rudder (Daniel: configured as a
        # normal airplane, aileron = rotor tilt)
        msp.set_servo_mixer_rule(0, 0, IN_ROLL)     # rotor tilt
        msp.set_servo_mixer_rule(1, 1, IN_PITCH)    # elevator
        msp.set_servo_mixer_rule(2, 2, IN_YAW)      # rudder
        return 3
    # QHS / QHS_FLAPS
    msp.set_servo_mixer_rule(0, 0, IN_ROLL)
    msp.set_servo_mixer_rule(1, 1, IN_PITCH)
    msp.set_servo_mixer_rule(2, 2, IN_YAW)
    if actuators == "QHS_FLAPS":
        # flap servo with the decided slow deployment (speed 50 = 2 s full
        # travel, the aerodynamically gentle rate). Driven by the FLAPERON
        # box on a real build; the bench show-flight commands the plant
        # flaps directly (a pilot channel, not a stabilized output).
        msp.set_servo_mixer_rule(3, 3, IN_FLAPS, rate=100, speed=50)
        return 4
    return 3


def provision_model(model):
    actuators, repertoire = AIRFRAMES[model]
    print(f"=== {model}: {actuators} -> {', '.join(repertoire)}")
    bench.provision()             # core settings + save (SITL reboots)
    time.sleep(3)
    msp = MspClient()
    n = provision_mixer(msp, actuators)
    msp.set_servo_mixer_rule(n, 0, 0, rate=0)   # terminate the rule list
    if model == "binary":
        # the Binary carries a real pitot (Daniel) - FAKE driver, the
        # HITL stream injects the truth airspeed
        msp.set_setting("pitot_hardware", struct.pack("<B", 5))
    if actuators == "GYRO":
        # the gyro flies NO attitude presets and no figure bands - rebuild
        # the mode map from scratch: ARM, ANGLE, FLOOR, ROTOR GUARD (perm
        # 80, alone on the SELECT channel - the bench core would otherwise
        # leave P-HANG overlapping at 1870+, measured as a phantom hang
        # box in the tip test). Empty ranges clear the remaining slots;
        # the FC compacts the list at the first gap on save.
        msp.set_mode_range(2, 73, bench.CH_INVERTED - 4, 1700, 2100)  # FLOOR
        msp.set_mode_range(3, 80, bench.CH_SELECT - 4, 1700, 2100)
        for slot in range(4, 12):
            msp.set_mode_range(slot, 0, 0, 900, 900)
    msp.save_eeprom()
    msp.close()
    time.sleep(3)
    if os.path.exists("fcdata/eeprom.bin"):
        shutil.copy("fcdata/eeprom.bin", f"eeprom_{model}.bin")
        # completion marker: a provisioning killed mid-way leaves a HALF
        # image, and restoring one flew the timber with a broken config
        # to 314 m (measured) - only marker-verified images get restored
        with open(f"eeprom_{model}.bin.ok", "w") as fh:
            fh.write("complete\n")
        print(f"-> eeprom_{model}.bin (+marker)")


def main():
    which = [a for a in sys.argv[1:] if a in AIRFRAMES]
    if not which:
        print(__doc__)
        raise SystemExit(f"models: {', '.join(AIRFRAMES)}")
    for model in which:
        provision_model(model)


if __name__ == "__main__":
    main()
