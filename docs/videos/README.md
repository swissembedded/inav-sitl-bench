# Flight replay videos - what you are seeing

Every video is a REAL closed-loop flight: the actual INAV firmware (SITL)
flies a JSBSim physics model through the MSP simulator interface, driven by
scripted RC inputs. Nothing is animated by hand.

Each replay shows:

- **3D view** - the aircraft's true (physics) attitude and flight path,
  with the mode readback from the FC itself in red (pulled via MSP, not
  re-derived from the RC script).
- **Roll / pitch graphs** - solid = physics truth, dashed = the FC's own
  attitude estimate. When the loop is honest they overlap.
- **Altitude / IAS / thrust** on the second axis; floor demos draw the
  floor line.
- **controller IN** - the pilot's stick positions (the script's commands).
- **switches** - the mode levers exactly as provisioned on the FC
  (labels = FC box names; the FLOOR lever has its own switch).
- **controller OUT** - what the FC drives onto the actuators the airframe
  REALLY has (elevons shown as the FC's own mix; flaps; TVC nozzle;
  rotor tilt on the gyro).

The 19 `*_show` videos share one format: arm at 25 m, a short trim phase
where the controller finds the airframe's level throttle by itself
("Einflug"), then only the figures this airframe can physically fly (from
its actuator set and power), each entered from a horizontal base line -
and every show CLOSES with one explicit safety-floor test: a held dive
into the floor line, caught by the firmware against the held stick. The
FLOOR switch is on for the entire flight. Sensor suite: GPS + magnetometer
+ baro (the Binary also carries a pitot), all fed with physics truth.

## Airframe shows (one video per model)

- `jsbsim_aerobat3d_show.mp4` - the reference 3D plane (aileron/elevator/
  rudder, T/W well above 1). Full repertoire: inverted hold, axial roll,
  loop, knife edge both sides, held flat spin, prop hang.
- `jsbsim_turbotimber_show.mp4` - Turbo Timber Evolution on 4S (T/W ~2,
  slotted flaps). Adds the blown-flap harrier: flaps deploy slowly (2 s
  servo), then a deep high-alpha pass rides the prop wash; the hang shows
  the overpowered climb-out temperament.
- `jsbsim_kingfisher_show.mp4` - Kingfisher on 3S: barely no hover
  (T/W 0.86), so no hang in the repertoire - inverted, roll, loop, knife,
  spin and the flaps harrier.
- `jsbsim_dragonfly_show.mp4` - pusher with elevons + rudder: no flaps,
  fast knife instead of the slow one, no hang.
- `jsbsim_funjet_show.mp4` - pusher delta with 2-axis thrust vectoring:
  elevons only in forward flight; the hang hovers on the NOZZLE (watch
  the tvc bars in controller OUT while the elevons barely matter at zero
  airspeed - the intake flow keeps the slow rotation governed).
- `jsbsim_easyglider_show.mp4` / `jsbsim_easystar_show.mp4` /
  `jsbsim_aeroscout_show.mp4` / `jsbsim_icona5_show.mp4` /
  `jsbsim_pt17_show.mp4` - the gentle trainers/scale ships: reduced
  repertoires (no hang - not enough power; the PT-17 biplane is the
  slowest climber in the fleet, watch the long patient transits).
- `jsbsim_bf109_show.mp4` - warbird: inverted, roll, loop, fast knife.
  NO spin on purpose: the narrow wing with its sharp stall break cannot
  HOLD a flat spin - it tumbles (measured, twice).
- `jsbsim_mig15_show.mp4` / `jsbsim_vampire_show.mp4` - EDF jets with
  aileron + elevator ONLY (fixed fins, no rudder servo): the knife-edge
  boxes are not even offered by the firmware (servo-mixer capability
  gating) - inverted, roll, loop.
- `jsbsim_lippisch_show.mp4` / `jsbsim_arwing_show.mp4` /
  `jsbsim_deltastrike_show.mp4` / `jsbsim_xeno_show.mp4` - elevon-only
  flying wings/deltas (Delta Strike with a small fin rudder): watch
  controller OUT showing the FC's real elevon mix instead of fake
  aileron/elevator bars.
- `jsbsim_a10_show.mp4` - twin-EDF A-10 (2.3 kg, zero propwash): flies
  on airspeed alone; flaps slow-pass instead of harrier; the loop needs
  the visible pre-loop deceleration (apex = entry + 2v/omega).
- `jsbsim_binary_show.mp4` - twin-motor FPV platform with flaps AND a
  pitot: the airspeed the FC sees is measured, not estimated.

## Autogyro pair (same flight, with and without protection)

- `jsbsim_autog2_tip_manual.mp4` - Durafly Auto-G2: slow flight starves
  the rotor (lift and roll authority both go with rotor rpm squared);
  the retreating-blade stall wins, the gyro rolls away past recovery and
  goes in. The honest failure, protection off.
- `jsbsim_autog2_tip_guard.mp4` - the same starving with the ROTOR GUARD
  box on: every tip-over is caught (wings level, nose slightly down,
  throttle floor - thrust is the only lever that restores rotor rpm),
  and when the pilot returns the throttle the aircraft is FLYING.
- `jsbsim_autog2_tip_land.mp4` - the landing contract: guard armed, the
  pilot pulls the throttle to IDLE. The rotor starves and the gyro tips,
  but the guard stands down by design - an idle stick is landing intent,
  and the FC never raises the thrust against it (the panel shows the
  throttle output pinned at zero through the tip).

## Single-maneuver deep dives (reference airframe)

- `jsbsim_inverted.mp4` - inverted hold through a downdraft gust and a
  deliberate rudder turn; altitude assist keeps the height.
- `jsbsim_inverted_stick.mp4` - the stick SEMANTICS around a hold: half
  aileron is a HELD angle offset from inverted (not a rate), release
  returns gently; then the same on the elevator.
- `jsbsim_knife_left.mp4` / `jsbsim_knife_right.mp4` - knife edge holds:
  the fuselage carries the weight on the rudder, per-side pitch trim,
  altitude assist works the nose angle.
- `jsbsim_hang.mp4` - prop hang at ~90 deg: the controller finds its own
  hover throttle; heading is the free axis.
- `jsbsim_hang_tvc.mp4` - prop hang on the TVC pusher delta: ALL
  authority from the vectored nozzle (with inverse throttle
  compensation), elevons dead at zero airspeed.
- `jsbsim_loop_fig.mp4` - commanded full loop closing on the entry
  altitude, level hold with assist afterwards.
- `jsbsim_roll_hold.mp4` - F ROLL: earth-referenced axial roll, the
  assist distributes nose-up onto elevator and rudder as the roll phase
  demands.
- `jsbsim_flat_spin.mp4` - FLAT SPIN box: attitude held flat while the
  pilot's full rudder drives the autorotation about the earth vertical;
  releasing the rudder stops the rotation, the attitude stays held.
- `jsbsim_inv_spin.mp4` - inverted flat spin (FLAT SPIN + INVERT): note
  the aircraft-referenced stick sense - seen from above the rotation
  reverses vs upright, like a real aircraft.
- `jsbsim_knife_spin.mp4` - knife-edge spin (FLAT SPIN + KNIFE L): the
  rudder command lands on the body pitch axis - the rotation the knife
  attitude leaves free.

## Safety floor demos (legacy - predictive engage era)

These three were flown under the earlier predictive floor law; the
current firmware engages on BREAKING THROUGH the line instead (no
prediction - a piloted trajectory is not predictable). They will be
re-shot in the final cut; the catch behavior itself looks the same.

- `jsbsim_floor_dive.mp4` - held dive into the floor, caught and leveled
  AGAINST the held stick; then the same dive with the floor off punches
  through.
- `jsbsim_floor_panic.mp4` - the panic case: throttle chopped, down
  elevator held - the catch brings its own climb throttle and suppresses
  the held stick.
- `jsbsim_floor_spin.mp4` - flat spin into the floor: the recovery
  overrides the spin, rolls upright out of the rotation and climbs out.

## Figure sequencer routines (F SEQ box)

Scripted aerobatic ROUTINES compiled into the firmware's figure
sequencer via MSP, then flown by flipping one switch - the FC owns the
whole trajectory (line-hold about the entry heading included):

- `jsbsim_seq_immelmann.mp4` - Immelmann turn.
- `jsbsim_seq_veloxity_3d_demo.mp4` - a 3D demo routine.
- `jsbsim_seq_wargo_addiction_xl.mp4`,
  `jsbsim_seq_wargo_vol8_immelmann_inverted.mp4`,
  `jsbsim_seq_wargo_gyro_knife_pass.mp4`,
  `jsbsim_seq_wargo_vol9_easy_routine.mp4` - routines transcribed from
  Joe Wargo's 3D instruction flights.
- `jsbsim_seq_chain.mp4` - three routines in ONE flight: the bench
  reprograms the sequence between legs over MSP, the F SEQ box edge
  restarts each.
