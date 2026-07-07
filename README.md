# INAV SITL bench — quaternion orientation hold

Closed-loop test framework for the `feature/quaternion-attitude-hold` branch
of [INAV](https://github.com/swissembedded/inav). Plays through all
orientation-hold targets (inverted,
knife edge left/right, prop hang) against the real INAV firmware running as
SITL, using the stock HITL sensor-injection path (`MSP_SIMULATOR` v3,
`HITL_USE_IMU`): synthetic gyro/acc from a rigid-body model -> real AHRS ->
real controller -> mixer outputs -> back into the model.

## Run

```
# 1. SITL container (SITL.elf built from the inav feature branch, e.g. via podman ubuntu:24.04)
#    (MSYS_NO_PATHCONV=1 is only needed in a Windows Git Bash / MSYS shell)
MSYS_NO_PATHCONV=1 podman run -d --name inav-sitl -p 5760:5760 \
  -v "<path-to>/inav:/src" -v "<path-to>/inav-sitl-bench/fcdata:/work" \
  -w /work ubuntu:24.04 /src/build_sitl_linux/bin/SITL.elf

# 2. one-time FC provisioning, then restart
python bench.py provision && podman restart inav-sitl

# 3. tests
python bench.py smoke       # sensor conventions + arming
python bench.py scenarios   # all orientation targets + ANGLE bailout
```

## Files

- `msp.py` — minimal MSPv2/TCP client (settings, mode ranges, servo mixer)
- `hitl.py` — `MSP_SIMULATOR` v3 payload pack/unpack
- `dynamics.py` — rigid body + doc consistency equations (gyro = body rates,
  acc = rotated gravity), INAV quaternion conventions
- `bench.py` — provisioning, smoke test, scenario runner (`scenario_log.csv`)

## SITL/HITL gotchas (cost hours, do not rediscover)

1. **Gyro boot calibration freezes forever** if the first `MSP_SIMULATOR`
   frame arrives before it finishes (`gyroUpdate()` early-returns under
   HITL; `USE_IMU_FAKE` compiles out the `init_gyro_cal` bypass on SITL).
   Wait for armingFlags bit 9 to clear before enabling HITL.
2. `baro_hardware = FAKE` never completes calibration on SITL -> use NONE.
3. `receiver_type = SIM` only takes effect after reboot.
4. AHRS runs boosted acc gain for ~20 s after boot; gyro-integration checks
   read low during that window.
5. **No smix rules -> stabilized outputs stay 0** in the `MSP_SIMULATOR`
   reply (`simulatorData.input[]` is only written inside `servoMixer()`,
   which only runs when `isMixerUsingServos()`).
6. SITL keeps state between bench runs — always settle the AHRS to the
   model attitude adaptively, never with a fixed delay.

## Commands

- `python bench.py provision` — one-time FC setup (run against a freshly
  restarted SITL, then restart again)
- `python bench.py smoke` — sensor conventions + arming
- `python bench.py scenarios` — all orientation targets + ANGLE bailouts
  (inverted bailout is known-flaky in stock Euler ANGLE: rolling back from
  exactly 180 deg picks a random direction)
- `python bench.py edge` — antipode starts, pitch-90 crossing
- `python bench.py floor` — altitude floor: climb, dive, catch above the
  floor, then landing descent with the box off stays untouched
- `python bench.py tvc` — TVC/surface deflection ratio vs thrust
  (inverse compensation: ~1 at full thrust, ~4 at the idle cap)
- `python bench.py sequence` — programmed chain with precondition gate:
  WAIT_ALT 40 m -> Immelmann (half loop + half roll) -> hold; gate must be
  respected, ends upright holding the gained altitude

## Gotcha 7 (cost a full debug loop)

The plant's nose-in-earth mapping must be the INVERSE of the (FC-validated)
gravity mapping: `rotate_earth_to_body(qconj(q), x)` with z negated.
`rotate_earth_to_body(q, x)` is only correct at yaw 0 and flips the climb
sign at heading 180 — after every Immelmann the plane sank nose-up while
the FC was right all along (and the bench's own elevation logs lied the
same way). Symptom to remember: constant FC-vs-plant attitude offset that
"never corrects" = suspect the BENCH world model, not the firmware.

## TODO

- Full TVC plant term (tau = thrust x lever x sin(vane angle)) for a
  closed-loop hover scenario — the tvc check verifies the mixer path only.

## License

GPL-3.0-or-later — see [LICENSE](LICENSE).

Portions of the quaternion/rotation math (conventions of
`imuComputeQuaternionFromRPY`, `quaternionRotateVector`,
`axisAngleToQuaternion`) are derived from
[INAV](https://github.com/iNavFlight/inav), itself GPL-3.0-or-later.
