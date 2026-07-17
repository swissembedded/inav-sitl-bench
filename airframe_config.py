# Copyright (C) 2026 Daniel Haensse
# GPL-3.0-or-later (see repo LICENSE)
"""Single source of truth for the hangar: per airframe the ACTUATORS it
really has (Daniel-verified 2026-07-16), and the capability-derived
maneuver repertoire. Everything else derives from this file: the FC
provisioning (mixer layout), the replay panel's controller-OUT bars, and
the one-video-per-airplane sequence (no maneuvers an airframe obviously
cannot fly).

Actuator sets:
  QHS        aileron, elevator, rudder
  QHS_FLAPS  QHS + flaps
  QH         aileron + elevator only (no rudder)
  ELEVON     elevons only
  ELEVON_R   elevons + rudder
  ELEVON_TVC elevons + thrust vectoring (funjet)
  GYRO       elevator, rudder, lateral rotor tilt (+ pre-rotator)
"""

# capability flags derived per actuator set / power:
#   knife needs a rudder; hang needs propwash authority AND T/W >= ~1;
#   inverted needs enough negative CL; spins need yaw authority.
AIRFRAMES = {
    #             actuators     repertoire (in sequence order)
    "aerobat3d": ("QHS",        ["inverted", "roll", "loop", "knife", "spin", "hang"]),
    "funjet":    ("ELEVON_TVC", ["inverted", "roll", "loop", "hang"]),
    "turbotimber": ("QHS_FLAPS", ["inverted", "roll", "loop", "knife", "spin",
                                  "flaps_harrier", "hang"]),
    "kingfisher": ("QHS_FLAPS", ["inverted", "roll", "loop", "knife", "spin",
                                 "flaps_harrier"]),
    "dragonfly": ("ELEVON_R",   ["inverted", "roll", "loop", "knife_fast"]),
    "easyglider": ("QHS",       ["inverted", "roll", "loop", "spin"]),
    "easystar":  ("QHS",        ["roll", "loop"]),
    "xeno":      ("ELEVON",     ["inverted", "roll", "loop"]),
    "aeroscout": ("QHS",        ["roll", "loop", "spin"]),
    "a10":       ("QHS_FLAPS",  ["inverted", "roll", "loop", "knife_fast",
                                 "flaps_slow"]),
    "icona5":    ("QHS",        ["roll", "loop"]),
    # NO spin: the narrow warbird wing with its sharp post-stall break
    # cannot HOLD a flat spin - it tumbles and eats 90 m in 5 s (measured,
    # consistent with the old batch where bf109 flat_spin crashed too)
    "bf109":     ("QHS",        ["inverted", "roll", "loop", "knife_fast"]),
    "lippisch":  ("ELEVON",     ["inverted", "roll", "loop"]),
    "mig15":     ("QH",         ["inverted", "roll", "loop"]),
    "pt17":      ("QHS",        ["roll", "loop", "spin"]),
    "binary":    ("QHS_FLAPS",  ["roll", "loop", "flaps_slow"]),
    "arwing":    ("ELEVON",     ["inverted", "roll", "loop"]),
    "deltastrike": ("ELEVON",   ["inverted", "roll", "loop"]),
    "vampire":   ("QH",         ["inverted", "roll", "loop"]),
    # gyro: ground start IS the test (breaks out LEFT under acceleration:
    # hold right tilt + up elevator, pre-rotator to full rpm first, one-way
    # bearing spins the rotor up further with airspeed); flow rule: rotor
    # lift ~ rpm^2, rpm lives on inflow - climbing bleeds it.
    # PLUS the tip-over pair (floor_dive pattern, separate videos):
    # tip_manual = slow flight without controller, rotor decays, tilt goes
    # soft, gyro rolls away; tip_recovery = same entry with the controller
    # holding attitude and the thrust governor restoring inflow -> rpm ->
    # authority. If the existing controller cannot catch it, that is the
    # measured case for the tip-monitor FW follow-up.
    "autog2":    ("GYRO",       ["ground_takeoff", "turns", "steep_descent",
                                 "half_loop_90"]),
}

# controller-OUT bars per actuator set (throttle always first)
PANEL_BARS = {
    "QHS":        ["throttle", "rudder", "elevator", "aileron"],
    "QHS_FLAPS":  ["throttle", "rudder", "elevator", "aileron", "flaps"],
    "QH":         ["throttle", "elevator", "aileron"],
    "ELEVON":     ["throttle", "elevon L", "elevon R"],
    "ELEVON_R":   ["throttle", "rudder", "elevon L", "elevon R"],
    "ELEVON_TVC": ["throttle", "elevon L", "elevon R", "tvc yaw", "tvc pitch"],
    # NO pre-rotator bar: the start motor is not modeled yet (brushed, no
    # telemetry) - a dead always-zero instrument is dishonest display;
    # the bar returns with the ground-takeoff work that models it
    "GYRO":       ["throttle", "rudder", "elevator", "rotor tilt"],
}


# Auto-G2 rotor model assumption (Daniel 2026-07-16): v1 starts WITHOUT a
# pre-rotator model - the rotor simply begins with an initial rotation.
# Coupling design: the FDM's disk lift is multiplied by the square of a
# normalized rotor-rpm property (like fcs/flap-cmd-norm, written by the
# plant each step); the plant integrates rpm from inflow (forward speed
# through the tilted disk, one-way bearing: airflow only spins it UP).
GYRO_ROTOR = dict(
    disk_diameter_m=0.821,
    rpm_nominal=450.0,       # Daniel-recherchiert: 400-500 rpm halten die
                             # Hoehe (Blattspitze ~19 m/s - leichte Foam-
                             # Scheibe, 1.1 kg/m2); Gas -> Fahrt -> Inflow
                             # -> Drehzahl ist der einzige Hoehen-Hebel
    rpm0_frac=0.7,           # pre-rotator: ~60-70% der Flugdrehzahl,
                             # akkuabhaengig, keine Ziel-RPM (brushed)
    rpm_min_frac=0.4,        # below this the disk stops carrying (tip-over
                             # regime - the monitor case)
)
