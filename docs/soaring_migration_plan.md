# Thermal Soaring for INAV — Migration Plan

Status: concept, agreed 2026-07-19. Owner: Daniel. Reference implementation:
ArduPilot `AP_Soaring` (Samuel Tabor) — used as a **specification**, not a
source to copy from.

## 1. Decision (the scope, as set by Daniel)

- **Autonomous thermal soaring becomes an INAV feature.** Motor off, ride the
  thermal, gain height/time.
- **Clean-room rewrite from scratch**, so it fits INAV natively — *not* a port
  of the ArduPilot C++. ArduSoar and Tabor's paper are the open reference for
  the algorithm; the code is ours.
- **Independent of the 3D aerobatics work.** Own feature flag, own PR. No shared
  logic with orientation-hold; the two are never active at the same time.
- **Pitot is a given.** A soaring airframe carries an airspeed sensor, so the
  total-energy vario is textbook-clean (no GPS/wind fallback needed).
- **Thermal loitering with a wind-shifted circle centre** is the core mechanism
  (section 3).

## 2. Licence basis (verified 2026-07-19)

- INAV = GPLv3 (top-level LICENSE; individual files mixed GPL-only and
  MPL-2.0-OR-GPL-3.0 dual from the Betaflight lineage).
- ArduPilot = GPLv3.
- The two are licence-**compatible**: a port with attribution would be legal.
  The rewrite is chosen for **hygiene and provenance**, not legal necessity —
  it keeps the files clean of a copied-from-ArduPilot GPL-only header and
  avoids the cross-project-copy discussion in the upstream PR.

## 3. The centering law (Daniel's specification)

Fly a loiter circle; measure the energy gain around the circle; shift the
circle centre slowly toward maximum energy — and slide the whole circle with
the wind so it stays over the drifting thermal.

**Signal — net (total-energy) variometer.** With the pitot present:

```
e            = h + v^2 / (2 g)          specific total energy
net_vario    = d/dt(e) - expected_sink(v, bank)
```

The pitot lets the total-energy compensation remove airspeed transients (the
phantom climb on pull-out that a baro-only vario shows). This is the exact
signal ArduSoar is designed around.

**Direction — sin/cos correlation over the circle phase.** Rather than hunting
the single point of strongest lift (noisy), correlate net_vario against the
circle azimuth psi over one revolution — the first harmonic points straight at
the strongest climb and averages the noise:

```
dGx  ∝  Σ (net_vario_i - mean) · cos(psi_i)
dGy  ∝  Σ (net_vario_i - mean) · sin(psi_i)
```

**Centre update — gradient PLUS wind drift (the wind-shifted curve).** A thermal
is bound to the air mass and drifts with the wind; a GPS-fixed circle would let
it escape downwind. Per update (once per revolution / half-revolution):

```
Centre += k_grad · (dGx, dGy)  +  wind · dt
```

- `k_grad · gradient`  centres the circle inside the thermal
- `wind · dt`          slides the circle with the drifting air mass

`k_grad` small + a low-pass on the gradient vector: "slowly", so the circle
does not chase noise and ealt/eier. INAV's wind estimator supplies `wind`.

**Circle geometry.** Fixed moderate bank to start (~35 deg); `R = v^2 / (g
tan phi)` sets the radius. Adaptive radius/bank is a later refinement.

**Entry / exit.** Enter when net_vario exceeds a threshold; exit when it falls
below a lower threshold (thermal flown through / topped out) or an altitude
band is reached, then resume cruise / motor on. Simple thresholds first;
MacCready speed-to-fly (section 6) is the refined version.

## 4. INAV building blocks reused (already present)

- **Wandering forced-poshold anchor** — built for the floor orbit
  (`navActivateFloorOrbitAt` / `navAssertFloorOrbitTarget`, re-anchored per RX
  cycle). This is exactly the "shift the loiter centre every cycle" machinery
  the centering law needs; generalise it into a shared nav helper.
- **Wind estimator** — supplies the `wind` term for the drift shift.
- **Airspeed / pitot** (`USE_PITOT`, `pitot_hardware`) — the vario input;
  already provisioned in the bench (the `binary` cell flies with `--pitot`).
- **Baro + INS** — altitude and energy state.
- **Nav loiter / poshold FSM**, `MOTOR_STOPPED_USER` throttle path (touched
  during crash-detection work).

## 5. New module: `flight/soaring.c` (+ `.h`)

Thin, guidance-layer, no AHRS/PID changes:

- net-vario computation from energy state
- centering law (sin/cos gradient + wind drift shift)
- soaring state machine (cruise / thermalling), entry-exit thresholds
- throttle-cut hook (motor off while soaring, on when cruising/aborting)
- a SOARING nav state/box that drives the wandering loiter anchor onto the
  estimated thermal centre
- parameters via PG + settings.yaml (`SOAR_*` mapped to INAV naming)
- SITL debug slot for the bench (thermal-centre estimate, net-vario, state)

Gating: `#ifdef USE_SOARING` compile gate + a runtime feature bit, same
pattern as the aerobatics suite, so the feature is fully switch-off-able and
the core loop is byte-identical with it off.

## 6. Bench prerequisite (do first)

We have no thermal model in the jsbsim plant yet. Build one:

- **thermal field**: Gaussian lift columns `w(r) = w0 · exp(-r^2/R^2)`, a few
  placed in the arena, each drifting with a configurable wind, plus turbulence.
- couple the updraft correctly into the **energy balance** (the plant already
  outputs IAS; the updraft adds vertical air velocity, it does not directly
  change the pitot reading — the energy shows up in net-vario).
- **centering gates** (our real advantage over ArduPilot): e.g. "in a thermal
  of strength X the circle centres within N revolutions to < R m of the true
  centre", "with wind Y the loiter does not lose the thermal", "net-vario is
  transient-free on the pull-out".

## 7. Phases

1. **Bench thermal model + net-vario** — plant field, energy coupling, the
   total-energy vario computed from IAS+baro+INS; verify the vario is clean.
2. **Centering law on the wandering anchor** — sin/cos gradient + wind shift,
   flown in SITL, passed by the centering gates. This is the heart.
3. **State machine** — cruise/thermalling transitions, entry/exit thresholds,
   throttle cut, altitude-band and lost-thermal failsafes.
4. **Parameters + GUI** — `SOAR_*` as PG/settings.yaml, configurator fields.
5. **Refinements (optional / later)** — 4-state thermal EKF as an upgrade path
   beside the gradient centerer; MacCready speed-to-fly for cross-country.

Estimate: heart (phases 1-2) ~1-2 weeks to a SITL-soaring proof; robust
PR-ready (phases 3-4) ~3-5 weeks. Smaller than the 3D rewrite and cleaner
(pure guidance layer).

## 8. Where we aim to beat ArduSoar

- **Bench + adversarial gates** — systematic centering/robustness tests
  ArduPilot does not have at this depth.
- **Explicit wind-drift compensation** — the `wind · dt` centre shift keeps the
  circle over the drifting thermal directly, instead of leaning on the EKF to
  chase it implicitly.
- **A simple, legible default centerer** — the sin/cos gradient law as the
  robust default, with the EKF as an optional upgrade (ArduSoar is EKF-only).

Where ArduPilot stays ahead (not glossed over): years of real-flight tuning,
mature mode transitions and failsafe detail, and cross-country MacCready. The
pitot removes the one physics disadvantage.

## 9. ArduSoar weaknesses found in the source (2026-07-19) — build the fixes in

Read of `AP_Soaring.cpp`, `Variometer.cpp`, `ExtendedKalmanFilter`:

1. **Wind-drift model is physically wrong (the big one).** ArduSoar shifts the
   thermal estimate by `wind_drift = wind * dt * filtered_climb / ekf.X[0]` -
   wind scaled by (climb rate / thermal strength). No physical basis, and it
   blows up as the thermal weakens (X[0] -> 0). **Our fix:** a thermal is
   locked to the air mass and drifts at exactly the wind speed - `Centre +=
   wind * dt`, unscaled. This is precisely Daniel's wind-shifted circle, and it
   is both correct and robust in weak lift. Biggest single improvement.

2. **Vario sink term uses a small-angle cosine approximation** `cosphi ~ 1 -
   phi^2/2`, only valid for shallow bank - but thermalling bank is 35-45 deg,
   where the `1/cos^2(phi)` sink term the approximation feeds is meaningfully
   off. **Our fix:** exact `cos(phi)`. Trivial, and accurate at the bank angles
   that actually matter.

3. **No measurement gating on the vario.** The EKF ingests every reading, no
   outlier rejection - a gust spike walks the estimate. **Our fix:** reject
   vario outliers before they move the centre (fits our gate philosophy).

4. **MacCready speed-to-fly is stubbed out** - `McCready()` returns just
   `thermal_vspeed` with a "method shell to be filled in later" comment. So
   ArduSoar's cross-country speed optimisation is incomplete. **Our chance:**
   implement it properly from the polar (an altitude/expected-climb curve) -
   genuine value beyond ArduSoar. Optional / phase 5.

5. **EKF init assumes the thermal is dead ahead** (fixed distance on the yaw
   vector), missing offset thermals; initial radius and covariance hardcoded.
   **Our fix:** the sin/cos gradient centerer needs no position prior - it
   measures the gradient and walks there, robust to an offset thermal. If we
   add the EKF as an upgrade, seed it from the first circle's gradient.

Improvements 1 and 2 fold straight into the phase-2 centering law; 3 is a small
guard; 4 and 5 are the EKF/MacCready upgrade path (phase 5). None of them touch
3D-flight code - this stays a standalone feature/PR.

## 10. Pitot compensation in turns (Daniel, must not forget)

Circling loads the net vario, and the pitot does not read the CG airspeed: the
tube sits at position **r relative to the CG**, so it measures `v_cg + omega x r`
along its axis. With the yaw rate omega_z dominant in a circle:

```
v_measured ~= v_cg - omega_z * r_y        (r_y = lateral offset of the tip)
```

so a **laterally offset** pitot reads high on one side of the circle and low on
the other - left vs right is the sign of r_y. A tip on the centreline (r_y = 0)
has no error in a steady circle. In a uniform circle omega_z and r_y are
constant, so the error is a DC offset the vario-mean removes (centering stays
robust), but absolute vario accuracy for the trigger/exit thresholds and any
non-uniform turn suffer.

Build it twice:
- **bench plant**: model the measured pitot speed as `v_cg + omega x r_pitot`
  (parameter: pitot tip offset from CG, esp. the lateral component, signed) so
  the bench is realistic and the FW correction is testable;
- **FW**: compensate `v_cg = v_measured + omega_z * r_y`, parameter the lateral
  pitot offset [cm], signed = left/right.

Vector is pitot-tip-relative-to-CG; the lateral (y-body) term dominates in a
thermal circle, the longitudinal term only matters for the pitch-rate component.

## 11. Progress (this session)

- Module `flight/soaring.c/.h` built, USE_SOARING gated, committed
  (net vario with exact-cos sink, sin/cos gradient + wind*dt centering, state
  machine, 7 SOAR_* params). inav 10ac09200.
- Nav-anchor generalised: the wandering forced-poshold anchor renamed
  floorOrbit* -> navForcedPoshold*, gated USE_ORIENTATION_HOLD || USE_SOARING,
  so floor and soaring share it (no rename of USE_ORIENTATION_HOLD itself).
  Soaring drives the loiter onto the thermal-centre estimate through it.
- TODO next: motor cut while circling; bench thermal-centering gate (does the
  estimate converge on the true thermal?); pitot-in-turns (section 10);
  configurator SOARING mode + net-vario OSD/telemetry.
