# Experimental firmware

Prebuilt hex files of the quaternion orientation-hold branch, so testers
can flash without a toolchain. **Experimental - not flight-tested.** The
maiden flights are still ahead; treat these as bench/ground-test builds
and keep props off until you know your model.

- Source: `feature/quaternion-attitude-hold` @ `cf607ace9`
  (swissembedded/inav fork), INAV 9.1.0 base.
- Built out-of-tree, `cmake -DCMAKE_BUILD_TYPE=Release -G Ninja`.
- All six targets include `USE_ORIENTATION_HOLD` (F7/H7 class).

| Target | Notes |
| --- | --- |
| MATEKH743 | reference board for our own maiden flights |
| MATEKF722SE | |
| MATEKF405TE | |
| KAKUTEH7 | |
| SPEEDYBEEF745AIO | |
| MATEKF411 | fits, but tight - F411 has no room to spare |

Flashing: INAV Configurator -> Firmware Flasher -> "Load firmware
[Local]" -> pick the hex -> Flash. Full chip erase recommended when
coming from a different firmware.

Setup for the new modes lives in the bench repo docs
(`docs/rc_3d_flying_quick_guide.md`, the trimming checklist first).
