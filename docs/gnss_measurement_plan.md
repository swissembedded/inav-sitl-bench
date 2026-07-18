# GNSS false-valid gate - measurement plan

INAV's position estimator carries a disabled GPS acceptance gate
(`src/main/navigation/navigation_pos_estimator.c:730`):

    //const float gpsWeightScaler = scaleRangef(bellCurve(gpsPosResidualMag,
    //        INAV_GPS_ACCEPTANCE_EPE), 0.0f, 1.0f, 0.1f, 1.0f);
    const float gpsWeightScaler = 1.0f;

The residual it would act on is the difference between the GPS position and
the estimator's own position, in cm. `bellCurve(x, w) = exp(-x^2 / (2 w^2))`,
`INAV_GPS_ACCEPTANCE_EPE = 500` cm. The same pattern runs live in the
surface-offset gate (`navigation_pos_estimator_agl.c:149`). We want to arm
the GPS gate against the false-valid failure (`--gps-falsevalid` in
`jsbsim_fly.py`: shaded antenna, the receiver coasts its position with a
decaying stale velocity while keeping optimistic fix flags and a lagging
eph).

Method mandate: measure the residual distribution BEFORE setting any
threshold. This plan says which flights to fly, which numbers decide, and
what pass/fail means. Scope: the gate scales the XY weights only - the Z
path has no bellCurve gate, so GPS-altitude numbers below are context, not
gate inputs.

## What is measured

`gnss_residual_analysis.py` reads a flight CSV and reports two XY residuals
(magnitudes, per phase and overall):

- `xy_truth`: injected GPS position minus plant truth. On a `--gps` flight
  this is the noise floor of the injection chain (a few cm). On a
  `--gps-falsevalid` flight it is the size of the lie.
- `xy_inertial`: injected GPS position minus a simple inertially-propagated
  reference (anchored on the GPS fix, moved by the plant-truth displacement,
  re-anchored every 2 s). This stands in for the residual the estimator
  would see: the estimator pulls its position toward GPS with
  `w_xy_gps_p = 1.0`, a pull time of about 1 s, so a reference of mean age
  about 1 s sees the same residual scale. The proxy brackets the real
  residual within a factor of about 2; the armed-gate reflight is the end
  proof, not this analysis.

The prep phases (settle/cal/armL/armH) are excluded from all overall
numbers, same as the show gate in `_run_show_videos.py`. The tool
auto-excludes any file whose injected GPS leaves the truth by more than 2 m
from the healthy-width recommendation, so mixing files up cannot poison it.

## Flights to fly (phase 1: measure, gate stays disabled)

Aerobat3d, the standard sensor suite (`--gps --mag`), each flight twice:
once with truth GPS (baseline), once with `--gps-falsevalid`. If the
false-valid flag is implemented as a modifier on top of `--gps`, pass both;
the flag semantics follow the injection work.

| # | maneuver   | condition        | log copy                            |
|---|------------|------------------|-------------------------------------|
| 1 | roll_hold  | --gps            | jsbsim_log_roll_hold_gps.csv        |
| 2 | roll_hold  | --gps-falsevalid | jsbsim_log_roll_hold_falsevalid.csv |
| 3 | knife_left | --gps            | jsbsim_log_knife_left_gps.csv       |
| 4 | knife_left | --gps-falsevalid | jsbsim_log_knife_left_falsevalid.csv|
| 5 | show       | --gps            | jsbsim_log_show_gps.csv             |
| 6 | show       | --gps-falsevalid | jsbsim_log_show_falsevalid.csv      |

The singles cover a sustained aggressive figure each (the antenna tilts
through the roll and the knife edge - exactly where a real receiver starts
lying); the show covers the whole repertoire in one 190 s flight, floor on.

Per flight, house convention (fresh FC, saved aerobat3d image, log renamed
before the next flight overwrites it):

    python -c "import shutil; shutil.copy('eeprom_aerobat3d.bin', 'fcdata/eeprom.bin')"
    podman restart inav-sitl
    python -c "import time; time.sleep(4)"
    python jsbsim_fly.py --flip-ele --lockstep --model aerobat3d roll_hold --gps --mag
    python -c "import shutil; shutil.copy('jsbsim_log_roll_hold.csv', 'jsbsim_log_roll_hold_gps.csv')"

(substitute maneuver, GPS flag and copy name per the table; the show writes
`jsbsim_log_show.csv`).

## Analysis

    python gnss_residual_analysis.py jsbsim_log_roll_hold_gps.csv jsbsim_log_knife_left_gps.csv jsbsim_log_show_gps.csv
    python gnss_residual_analysis.py jsbsim_log_roll_hold_falsevalid.csv
    python gnss_residual_analysis.py jsbsim_log_knife_left_falsevalid.csv
    python gnss_residual_analysis.py jsbsim_log_show_falsevalid.csv

The first run prints the healthy-width recommendation; the other three
print, per false-valid flight, when the lie starts and when the gate would
have reacted.

## The numbers that decide

1. `W99` - the 99th-percentile `xy_inertial` residual over the three
   baseline flights combined (recommendation block). The gate width must
   satisfy `width >= W99 / 0.485` (0.485 x width is where the in-tree
   0.1..1.0 mapping reads 0.9). If that comes out at or below 500 cm, keep
   `INAV_GPS_ACCEPTANCE_EPE` unchanged - smallest change wins. The bench
   GPS is noiseless, so this measurement can only push the width UP, never
   justify shrinking it below 500 cm.
2. Detection latency - per false-valid flight, the time from "injected GPS
   leaves truth by 2 m" to "scaler would read <= 0.15" at the chosen width
   (printed by the tool).
3. Separation - healthy p99 vs the false-valid residual plateau. These must
   be far apart (different decades), otherwise a single width cannot both
   stay quiet and catch the lie, and the gate design has to be rethought
   instead of tuned.

## Acceptance: the gate does not hurt clean flight

On the three baseline flights, at the chosen width, prep phases excluded:

- A1: the in-tree scaler stays >= 0.9 for at least 99 percent of frames
  (tool: "gate view" line, and A1 is exactly the recommendation target).
- A2: no contiguous stretch longer than 0.5 s with the scaler below 0.5
  (tool: "longest stretch below 0.50").
- A3 (after arming, phase 2): the show reflight still passes the
  `_run_show_videos.py` gate, and the fused-altitude error `|fc_alt - alt|`
  matches the unarmed baseline (same p99 within 20 percent) - the armed
  gate must be invisible on a clean flight.

## Acceptance: the gate catches the false-valid case

On each false-valid flight, at the chosen width:

- B1: the lie exists - the injected GPS actually leaves the truth by more
  than 2 m during the shaded window (tool prints the leave time). A
  false-valid flight without a visible lie proves nothing.
- B2: detection latency (number 2 above) is at most 2 s. Physical basis: at
  the 20-30 m/s the aerobat flies, 2 s of coasting GPS is 40-60 m of
  residual - far outside any sane width.
- B3: the scaler stays <= 0.5 for as long as the residual exceeds the
  0.5-radius (6.4 m at 500 cm) - the weight must not flap up and down
  during the lie.
- B4: recovery - after the window, when the receiver reports honest
  positions again, the scaler returns to >= 0.9. In the phase-1 proxy this
  is immediate; on the armed FC the estimate has drifted meanwhile and
  re-acceptance works through the 0.1 floor slowly pulling the estimate
  back. Phase 2 must show recovery within 10 s of the window end, else the
  0.1 floor or the width needs revisiting.

## Phase 2 (arm and reflight)

Restore line 730 with the chosen width (keep the 0.1 floor - it is the
re-acceptance path), rebuild the SITL, refly all six flights with the same
commands, re-run the analysis, and judge A3/B4. Open point: the flight log
carries no FC XY position estimate (only `fc_alt`), so the XY catch on the
armed FC is only indirectly visible. Logging the estimator's local XY next
to the existing 10 Hz `fc_alt` poll closes that - one column pair, to be
added together with the arming.

## First sample (preliminary, 2026-07-18)

The injection work's roll_hold test log already on the bench contains a
short false-valid window and gives a first calibration of expectations -
to be redone on the final injection build before anything is decided:

- healthy segments: `xy_inertial` p90 about 0.10 m, p99 about 0.11 m -
  weight 1.000 at 500 cm; the 99-percent criterion A1 passes with two
  orders of margin.
- false-valid window: residual climbs to 28 m; the gate at 500 cm would
  have read scaler <= 0.15 within 0.68 s of the GPS leaving the truth
  (B2 passes), and healthy vs lie sit four decades apart (number 3 is
  comfortable).
