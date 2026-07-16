# Copyright (C) 2026 Daniel Haensse
# GPL-3.0-or-later (see repo LICENSE)
"""Per-airframe FC provisioning + check flight ("Einflug"), driven by
airframe_config.py. Produces eeprom_<model>.bin for the batch: mixer
matches the REAL actuator set (elevon mix where the airframe has
elevons), then a scripted check flight settles the learners before any
figure is flown. Only a PASSed check flight releases the airframe.

    python airframe_provision.py <model>     # provision + check flight
    python airframe_provision.py --all

Status: SKELETON - provision per actuator set implemented, check flight
TODO (rate-loop check, gain-learner settle, trim verify).
"""
import struct
import sys

from airframe_config import AIRFRAMES
from msp import MspClient

# INAV smix input sources
IN_ROLL, IN_PITCH, IN_YAW = 0, 1, 2


def provision_mixer(msp, actuators):
    if actuators.startswith("ELEVON"):
        # elevon mix: both servos carry roll AND pitch (FC-side mixing,
        # the honest config for xeno/arwing/deltastrike/lippisch/funjet)
        msp.set_servo_mixer_rule(0, 0, IN_ROLL, rate=50)
        msp.set_servo_mixer_rule(1, 0, IN_PITCH, rate=50)
        msp.set_servo_mixer_rule(2, 1, IN_ROLL, rate=-50)
        msp.set_servo_mixer_rule(3, 1, IN_PITCH, rate=50)
        if actuators == "ELEVON_R":
            msp.set_servo_mixer_rule(4, 2, IN_YAW)
        if actuators == "ELEVON_TVC":
            msp.set_servo_mixer_rule(4, 2, 62)   # TVC pitch
            msp.set_servo_mixer_rule(5, 3, 63)   # TVC yaw
    elif actuators == "QH":
        # no rudder on the airframe: no yaw rule at all - knife boxes
        # vanish via servoMixerHasYawControl(), exactly the FW gating
        msp.set_servo_mixer_rule(0, 0, IN_ROLL)
        msp.set_servo_mixer_rule(1, 1, IN_PITCH)
    elif actuators == "GYRO":
        # rotor tilt is lateral only -> roll; pitch = tail elevator
        msp.set_servo_mixer_rule(0, 0, IN_ROLL)    # rotor tilt
        msp.set_servo_mixer_rule(1, 1, IN_PITCH)   # elevator
        msp.set_servo_mixer_rule(2, 2, IN_YAW)     # rudder
    else:   # QHS / QHS_FLAPS
        msp.set_servo_mixer_rule(0, 0, IN_ROLL)
        msp.set_servo_mixer_rule(1, 1, IN_PITCH)
        msp.set_servo_mixer_rule(2, 2, IN_YAW)
        if actuators == "QHS_FLAPS":
            # INPUT_FEATURE_FLAPS with the decided 2 s deployment
            msp.set_servo_mixer_rule(3, 3, 14, rate=100, speed=50)


def main():
    which = ([a for a in sys.argv[1:] if a in AIRFRAMES]
             or (list(AIRFRAMES) if "--all" in sys.argv else []))
    if not which:
        print(__doc__)
        return
    for model in which:
        actuators, repertoire = AIRFRAMES[model]
        print(f"=== {model}: {actuators} -> {repertoire}")
        # TODO: base provisioning (bench.provision core) + provision_mixer
        # + per-type rates/PID scaling + check flight + eeprom_<model>.bin
        raise SystemExit("check flight not implemented yet - next step")


if __name__ == "__main__":
    main()
