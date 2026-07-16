# Experimental firmware

Prebuilt hex files of the quaternion orientation-hold branch, so testers
can flash without a toolchain. **Experimental - not flight-tested.** The
maiden flights are still ahead; treat these as bench/ground-test builds
and keep props off until you know your model.

- Source: `feature/quaternion-attitude-hold` @ `59638db1a`
  (swissembedded/inav fork), INAV 9.1.0 base.
- Built out-of-tree, `cmake -DCMAKE_BUILD_TYPE=Release -G Ninja`.
- Includes the derive-instead-of-ask rework (load governor
  `ohold_load_limit`, derived hover/assist throttle gains, derived crash
  threshold with boolean `crash_detection`, knife gating on the yaw
  effector) and the widened smix `speed` (uint16, CLI up to 1000 -
  declare your servo's real ceiling, e.g. 625 for a 0.08 s/60 deg
  MG92B; legacy MSP/configurator still shows at most 255).
- The feature needs more than 512 KB flash. F411 and F722 builds no
  longer carry it, so those two targets are gone from this list - flash
  a 1 MB+ board.

| Target | Notes |
| --- | --- |
| MATEKH743 | reference board for our own maiden flights |
| MATEKF405TE | |
| KAKUTEH7 | |
| SPEEDYBEEF745AIO | |

Companion configurator: `swissembedded/inav-configurator` branch
`release-9.1-ours` (the derived throttle gains no longer have GUI
fields; `ohold_load_limit` is CLI-only by design).
