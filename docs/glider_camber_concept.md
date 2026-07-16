# Glider Camber Control - Concept Note (No Work Item)

Status: **idea only, deliberately out of scope** for the orientation-hold
branch. Recorded so the thinking is not lost. Decision (2026-07-16):
whoever wants this on a glider can build it - **a pitot sensor is the
entry ticket**, and nothing in the orientation-hold work blocks it.

Source: community wishlist and a control-loop sketch collected from an
AI-assisted research session. Claims below are plausible but unverified -
re-research before any design decision.

---

## The community wishlist (context)

1. **Flight-phase camber** - thermal setting (flaps slightly down) /
   speed setting (slightly up). Today done with transmitter mixers,
   which fight the FC's stabilisation loop.
2. **Crow / butterfly landing mode** - the FC should know the flaps are
   a brake, apply the elevator compensation itself, and stop trying to
   regulate the pitching moment away.
3. **Thermal soaring** - vario-driven autonomous thermal centering
   instead of a fixed-radius loiter circle (exists in ArduPilot).
4. **Glider RTH** - come home at best-glide speed instead of holding
   altitude with elevator until the stall.
5. **Mixer for 4-6 control-surface wings** - full-span ailerons where
   the camber flaps follow aileron deflection.

## The control-loop sketch

Closed loop instead of fixed switch positions. Output enters the mixer
as a dynamic offset, which is INAV's existing structure anyway:

    servo = pilot input +/- stabilisation correction + camber offset

Three modules with different sensor demands:

| Module | Function | Needs |
|---|---|---|
| A | camber = f(airspeed), PI around the stored/learned polar | **pitot** |
| B | load compensator: more camber under g in tight thermal turns | IMU only |
| C | stall guard: flaps to neutral + push at the stall margin | pitot + IMU |

Notes that came out of the discussion:

- **A real pitot is a hard prerequisite for A and C.** INAV's virtual
  pitot (wind estimator + GPS) only converges while circling and is too
  slow and too coarse to close a loop around the polar.
- **Module B needs no pitot at all** - it is pure IMU load, the mirror
  image of the branch's g-load limiter (feed camber in instead of
  capping deflection).
- **No alpha vane needed.** With a pitot the stall margin follows from
  load: v_stall(n) = v_stall(1g) * sqrt(n). Pitot + accelerometer give a
  clean margin without extra sensing hardware.

## Honest staging

- without pitot: fixed flight-phase presets only (state of the art);
  module B possible today
- with pitot: modules A + C on top

## Relation to the orientation-hold branch

Adjacent mechanisms already exist there: the g-load limiter (module B's
mirror image), the slow flap rate with pitch compensation (the seed of
wishlist item 2), and the altitude floor (a relative of module C). A
glider cell with camber flaps in the bench hangar would make the loop
prototypable in SITL for free - the plant already logs true airspeed.
