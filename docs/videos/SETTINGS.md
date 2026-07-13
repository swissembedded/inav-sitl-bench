# One configuration for everything

Every video in this folder - all holds, spins, floor catches and every
figure sequence - flies on the SAME configuration: firmware defaults
plus the airframe-level provision below, applied once by
`bench.py provision`. No per-figure or per-video tuning, no `--set`
overrides in any video flight, and the eeprom is wiped before each run
so nothing learned in one flight leaks into the next.

The provision only contains what the trimming checklist
(`docs/rc_3d_flying_quick_guide.md`) tells a pilot to set once per
airframe, tuned once against the JSBSim aerobat3d plant:

| Setting | Value | Why (airframe-level) |
| --- | --- | --- |
| `fig_assist_z_gain` | 45 | altitude assist vs figure rate for this wing loading |
| `fig_assist_vz_gain` | 3 | dito, sink-rate term |
| `fig_assist_max` | 20 | assist authority cap |
| `ohold_knife_left_pitch_trim` | 7 | fuselage lift trim, left knife |
| `ohold_knife_right_pitch_trim` | 7 | fuselage lift trim, right knife |
| `inav_default_alt_sensor` | BARO_ONLY | bench flights inject no GPS |

Everything else (SIM receiver, AIRPLANE platform, fake baro, servo
mixer, mode switch ranges) is bench plumbing, not tuning.

The learned gains (hover damping, per-regime scales) start from their
defaults in every video; whatever adaptation you see happening IS the
shipped behavior, not preparation.
