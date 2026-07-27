# INAV PRs from the 3D-flight / soaring work

Snapshot 2026-07-19. Repos: `inav` (master = `4939a7ff`), `inav-configurator`
(`release-9.1-ours`), `inav-sitl-bench` (`main`). Push is Daniel's; this is the
map + push order.

> **PR-A (pitot lever-arm) was DROPPED (2026-07-19).** The correction is
> ~0.3 % at cruise, ~4 % even in a tight aerobatic turn — inside the pitot
> sensor's own error, and it cancels in soaring centering / is ~0 in straight
> knife-edge. Not worth the maintainer discussion. Removed everywhere:
> standalone branch deleted, aerobatics commit `21951f1dd` reverts it,
> configurator GUI dropped. The PR-A section below is kept only for the record.

## Standalone PRs (off master, independent)

### PR-A  Pitot lever-arm correction  — branch `feature/pitot-lever-arm`
- 1 commit `1931654eb` (off master, clean cherry-pick).
- `getAirspeedEstimate()` returns the CG airspeed: an offset tube reads
  `v_cg - yaw_rate*r_y` in a turn; corrected centrally in pitotmeter.c. New
  `pitot_lever_arm` [cm, signed, + = right]; default 0 = byte-identical.
  Novel - ArduPilot compensates IMU/GPS position but not the pitot.
- Sign validated analytically against imu.c centrifugal formula (inner side
  of the turn gets +). Smallest, cleanest PR - push first.
- OPEN: off-master compile; PR doc (calibration: measure CG->tip distance,
  optional left/right-circle cross-check).

### PR-B  Crash detection  — branch `feature/crash-detection`
- 3 commits `b11bd0854 / 1cad0c21f / 7645874cf` off master.
- USE_CRASH_DETECTION, motor-cut-after-impact-then-stillness, any platform.
- NOTE: this branch predates the feature-bit rework. The aerobatics branch has
  the newer version (`744f5bb18`: FEATURE_CRASH_DETECTION bit, decoupled from
  aerobatics, PG removed, GUI toggle). RECONCILE: bring `744f5bb18`'s feature
  bit + decoupling onto this standalone branch (or rebuild the branch from it).
- OPEN: reconcile to the feature-bit version, regenerate Settings.md, PR body.

## The big PR

### PR-C  3D aerobatics suite  — `#11695`, branch `feature/quaternion-attitude-hold`
- ~90 commits over master (orientation hold inverted/knife/hang/spin, altitude
  floor, figure sequencer, TVC, rotor guard, AHRS attitude/antipode gates,
  hook-diet, FW_AEROBATICS runtime feature, SITL lockstep harness).
- Partially pushed. The maintainers want it small - salami plan in
  `pr_slice_plan.md`.
- NOTE: crash (`744f5bb18`), soaring (`10ac09200..b8b08915e`) and pitot
  (`6856ccda8`) sit on top of this branch but belong to PR-B / PR-D / PR-A.
  They are already cleanly separable commits.
- OPEN: execute the salami slices, PR body v3, rebase on current master.

## Depends on aerobatics

### PR-D  Thermal soaring  — to extract from the aerobatics branch
- Commits `10ac09200` (module), `72aaffc0b` (nav-anchor generalise, SHARED with
  the floor orbit), `61050d928` (vario cal from the measured easyglider polar),
  `b8b08915e` (use PR-A's central pitot correction).
- USE_SOARING, net (total-energy) vario with exact-cos sink, sin/cos gradient
  + wind*dt centering, thermal loiter via the shared forced-poshold anchor.
- DEPENDS ON: PR-A (turn-clean vario) and the floor/nav-anchor (PR-C) - the
  anchor generalisation touches floor code.
- Status: builds, floor re-test green, NOT flying yet.
- OPEN: motor cut while circling; bench thermal-centering gate (does the
  estimate converge on the true thermal?); configurator SOARING mode + net-vario
  OSD; pitot lever-arm consumed (comes free via PR-A); extract to own branch.

## Companion changes (not FW PRs)
- `inav-sitl-bench` main (`d59f90a`): SITL lockstep harness, jsbsim plants +
  ThermalField, gates, the 46 canonical videos, polar/soaring tooling.
- `inav-configurator` `release-9.1-ours` (`beed194c`): FW_AEROBATICS +
  CRASH_DETECTION feature toggles and section-hiding.

## Suggested push order
1. **PR-A** pitot lever-arm - independent, tiny, clean.
2. **PR-B** crash detection - independent (after reconciling to the feature bit).
3. **PR-C** aerobatics `#11695` - the big one, sliced.
4. **PR-D** soaring - after PR-C (needs the nav-anchor) and PR-A.

## Cross-cutting open points
- Off-master compile of PR-A and PR-B (only verified inside the aerobatics build).
- Regenerate `docs/Settings.md` (crash setting removed, soaring + pitot params
  added) via `src/utils/update_cli_docs.py`.
- Aerobatics speed consumer: `knifeSpeedFF` still uses the throttle proxy; wire
  it onto the (corrected) pitot where present, attitude-gated - separate change
  in PR-C's branch.
- Configurator Electron runtime test of the new toggles (syntax-checked only).
