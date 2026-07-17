# Copyright (C) 2026 Daniel Haensse
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

# Closed loop: INAV Windows-SITL <-> JSBSim via the proven MSP_SIMULATOR path.
# Phases: settle AHRS -> arm -> ANGLE level (sign/convention check) -> INVERT.
# Usage: python jsbsim_fly.py [--flip-ele] [--flip-ail] [--flip-rud]
import sys, struct, time, math

from msp import MspClient
from hitl import sim_step
from jsbsim_plant import JSBSimPlant

FLIP_ELE = "--flip-ele" in sys.argv
FLIP_AIL = "--flip-ail" in sys.argv
FLIP_RUD = "--flip-rud" in sys.argv
# SITL started with --lockstep: every frame advances the FC clock exactly
# 1 ms, so the plant steps a fixed DT and no slot pacing is needed (the MSP
# roundtrip serializes the frames; the run may be faster than real time)
LOCKSTEP = "--lockstep" in sys.argv

RC_LOW, RC_MID, RC_HIGH = 1000, 1500, 2000
# --thr-scale <f>: scale EVERY phase's throttle toward idle (entry, holds,
# the MAN_RC per-maneuver values - all of it). The maneuver choreography is
# calibrated on the aerobat3d; an airframe with a different power loading
# flies the same script at a scaled throttle. Default 1.0 = untouched.
THR_SCALE = (float(sys.argv[sys.argv.index("--thr-scale") + 1])
             if "--thr-scale" in sys.argv else 1.0)
# channels: A E T R ARM ANGLE INVERT SELECT  (bench provisioning layout)
def rc_ch(thr=RC_LOW, arm=RC_LOW, angle=RC_LOW, floor=RC_LOW, sel=RC_LOW, ele=RC_MID, ail=RC_MID, rud=RC_MID):
    thr = int(round(1000 + (thr - 1000) * THR_SCALE))
    if FLOOR_ON and floor == RC_LOW:
        floor = 1900          # universal net: FLOOR switch on in every phase
    # angle = flight-mode selector (CH_ANGLE): FIGLOOP 1225 / FSPIN 1375 /
    #         FIGROLL 1525 / FIGSEQ 1675 / ANGLE >=1750
    # floor = FLOOR switch (CH_INVERTED), its own channel: >=1700 arms it
    # sel   = attitude-target selector (CH_SELECT): INVERT / KNIFE L/R / HANG
    return [ail, ele, thr, rud, arm, angle, floor, sel]

# Real active flight mode, pulled from the FC itself (not re-derived from RC):
# MSP_BOXIDS gives permanent box ids in active order, MSP_ACTIVEBOXES gives a
# bit per active box in the same order. Orientation-hold sub-modes win over
# ANGLE. permanentId map from src/main/fc/fc_msp_box.c.
MSP_BOXIDS = 119
MSP_ACTIVEBOXES = 113
MSP_ALTITUDE = 109
PERM_NAME = {69: "INVERT", 70: "KNIFE L", 71: "KNIFE R", 72: "P-HANG",
             74: "F ROLL", 75: "F LOOP", 76: "F 4PT", 77: "F SEQ",
             79: "FLAT SPIN", 80: "ROTOR GUARD"}

def fc_alt_m(m):
    p = m.request(MSP_ALTITUDE)          # int32 estimated altitude [cm], ...
    return struct.unpack("<i", p[:4])[0] / 100.0

def read_boxids(m):
    return list(m.request(MSP_BOXIDS))          # permanentId per active box

def fc_mode(m, boxids):
    bm = m.request(MSP_ACTIVEBOXES)
    active = {perm for i, perm in enumerate(boxids)
              if (i >> 3) < len(bm) and (bm[i >> 3] >> (i & 7)) & 1}
    if 27 in active:                            # FAILSAFE overrides everything
        return "FAILSAFE"
    # ALL active hold/figure boxes, combined: FLAT SPIN + an attitude
    # selector is a combined state (inverted flat spin etc.) and the
    # first-match display used to hide FLAT SPIN behind the attitude box
    names = [PERM_NAME[p] for p in (69, 70, 71, 72, 74, 75, 76, 77, 79)
             if p in active]
    base = "+".join(names) if names else None
    if base is None:
        base = "ANGLE" if 1 in active else ("ACRO" if 0 in active else "DISARMED")
    if 73 in active:                            # altitude floor engaged as suffix
        base += "+FLOOR"
    if 80 in active:                            # rotor guard armed as suffix
        base += "+GUARD"
    return base

FLAG_ARMED = 1 << 2
FLAG_CAL = 1 << 9

def arming_flags(m):
    p = m.request(0x2000)
    return struct.unpack("<I", p[9:13])[0]

def fc_att(m):
    p = m.request(108)
    r, pi, y = struct.unpack("<hhh", p[:6])
    return r / 10.0, pi / 10.0, y

m = MspClient()
# positional args, skipping option values (--set name=value)
def _positional(argv):
    out, skip = [], False
    for a in argv:
        if skip:
            skip = False
            continue
        if a in ("--set", "--imu-offset", "--model", "--start-m", "--gust-dir", "--thr", "--thr-scale"):
            # (--floor is a bare flag, no value to skip)
            skip = True
            continue
        if not a.startswith("--"):
            out.append(a)
    return out

_man = next(iter(_positional(sys.argv[1:])), "inverted")
_model = "c172p" if "--c172" in sys.argv else ("funjet" if _man == "hang_tvc" else "aerobat3d")
# --model <name>: fly any plant airframe (turbotimber, kingfisher,
# dragonfly) under the real FC instead of the maneuver's default
if "--model" in sys.argv:
    _model = sys.argv[sys.argv.index("--model") + 1]
# Every flight must stay within ~120 m: that is the legal RC ceiling and above
# it the aircraft is too small to see in the video. Each maneuver starts at
# (target apex - its own climb gain) so the peak lands at ~115 m. Descending
# maneuvers start near the top; climbers start low. The FLOOR demos ARM LOW so
# the floor (home + 30 m) becomes a low safety net the aircraft dives DOWN
# into - not a shelf 30 m above a 120 m start that forces a 250 m zoom-climb.
# An override is available: --start-m <metres>.
_M2FT = 3.281
_START_M = {
    "flat_spin": 108, "inv_spin": 104, "knife_spin": 104,
    "inverted": 104, "inverted_stick": 96, "knife_left": 103, "knife_right": 102,
    "hang": 68, "hang_tvc": 40, "loop_fig": 68, "roll_hold": 104,
    "crash_test": 104, "snap_neg": 104,
    "floor_dive": 25, "floor_panic": 25, "floor_spin": 25,   # arm low -> low net
    "seq": 120, "seq_chain": 120,   # aerobatic routines: exempt, they climb high
    "show": 25,   # one-video-per-airplane: arm low, Einflug, sequence
    "gyro_tip": 80,   # autogyro tip-over pair: the T/W-0.83 gyro cannot climb
                      # there itself - start high enough that TWO catch
                      # cycles fit above the terrain
}
_start_m = _START_M.get(_man, 104)
for _i, _a in enumerate(sys.argv):
    if _a == "--start-m":
        _start_m = float(sys.argv[_i + 1])
# --floor: the universal safety-net test. Arm LOW (25 m) like a real
# takeoff, climb scripted to the maneuver's altitude - the floor arms on
# the way up (floor 30 m + margin) - and keep the FLOOR switch ON through
# every phase. Any maneuver that descends through the net must be caught;
# the batch gate calls a miss below 20 m.
FLOOR_ON = "--floor" in sys.argv or _man == "show"   # show: net always on
_CLIMB_TARGET_M = _start_m
if FLOOR_ON and not _man.startswith("floor"):
    _start_m = 25.0
# 1500 ft for the diagnostic c172 (entry transient); everything else per map
plant = JSBSimPlant(model=_model,
                    alt_ft=1500 if _model == "c172p" else round(_start_m * _M2FT))
# --imu-offset x,y,z [m]: sensor lever arm from the CG (body frame). Off-CG
# sensors additionally measure w x (w x r) -- constant in the body frame
# during a steady spin, the false-down pull a CG-mounted model cannot show.
if "--imu-offset" in sys.argv:
    _off = [float(v) for v in sys.argv[sys.argv.index("--imu-offset") + 1].split(",")]
    plant.set_imu_offset(*_off)
    print(f"IMU lever arm: {_off} m")
log = open(f"jsbsim_log_{_man}.csv", "w")
log.write("t,phase,mode,fc_roll,fc_pitch,fc_yaw,js_roll,js_pitch,js_yaw,ias,alt,"
          "ail,ele,rud,thr,fc_thr,st_ail,st_ele,st_thr,st_rud,"
          "st_arm,st_angle,st_inv,st_sel,fc_alt,tvc_p,tvc_y,x,y,gps_fix,gps_sat,flap,"
          "safety\n")
BOXIDS = read_boxids(m)
_mode_cache = ["DISARMED", 0.0]     # [last mode string, last poll wall-time]
_alt_cache = [0.0]                  # last FC-measured altitude
_safety_cache = [0]                 # FW safety word (debug slot 7): bit0
                                    # floor armed, bit1 floor recovery,
                                    # bit2 rotor guard recovery
_gps_cache = [0, 0]                 # [fixType, numSat] from MSP_RAW_GPS
_div_max = [0.0, 0.0, ""]           # [max FC-vs-truth tilt divergence, time, phase]

def _tilt_div_deg(fr, fp, jr, jp):
    def up(r, p):
        r, p = math.radians(r), math.radians(p)
        return (-math.sin(p), math.sin(r) * math.cos(p), math.cos(r) * math.cos(p))
    a, b = up(fr, fp), up(jr, jp)
    return math.degrees(math.acos(max(-1.0, min(1.0, sum(x * y for x, y in zip(a, b))))))
# --set name=value (repeatable): write firmware settings live before the
# flight, value size taken from the FC's SETTING_INFO reply (sweeps).
# A value containing '.' is written as float32 raw bits (the estimator
# weights are f32 - integer bytes would corrupt them).
_argv = sys.argv[1:]
for _i, _a in enumerate(_argv):
    if _a == "--set" and _i + 1 < len(_argv) and "=" in _argv[_i + 1]:
        _n, _v = _argv[_i + 1].split("=", 1)
        _size = len(m.request(0x1003, _n.encode() + b"\x00"))
        if "." in _v:
            _raw = struct.pack("<f", float(_v))
        else:
            _raw = int(_v).to_bytes(_size, "little", signed=int(_v) < 0)
        m.request(0x1004, _n.encode() + b"\x00" + _raw)
        print(f"set {_n} = {_v}")

PARAMS = ["fig_roll_rate", "fig_loop_rate", "fig_assist_z_gain", "fig_assist_vz_gain",
          "fig_assist_max", "ohold_inverted_pitch_trim", "ohold_knife_left_pitch_trim",
          "ohold_knife_right_pitch_trim", "ohold_load_limit", "small_angle"]
with open(f"jsbsim_params_{_man}.txt", "w") as pf:
    for name in PARAMS:
        try:
            raw = m.request(0x1003, name.encode() + bytes([0]))
            pf.write(f"{name}={int.from_bytes(raw, 'little')}" + chr(10))
        except Exception:
            pass
# thrust vectoring: replicate the firmware's servo-mixer TVC inputs
# (INPUT_TVC_* = stabilized * thrustVectoringGain, thrust_vectoring.c)
def _u(name, default):
    try:
        return int.from_bytes(m.request(0x1003, name.encode()+bytes([0])), "little")
    except Exception:
        return default
_TVC_GAIN = _u("tvc_gain", 100) / 100.0
_TVC_COMP = _u("tvc_thrust_comp", 100) / 100.0
def tvc_gain(thr01):
    t = min(max(thr01, 0.25), 1.0)          # TVC_THRUST_COMP_FLOOR
    return _TVC_GAIN * (1.0 + (1.0/t - 1.0) * _TVC_COMP)

T0 = time.time()

# Fixed-timestep coupling: the plant advances exactly DT per cycle and the
# loop WAITS out the remainder of the slot, so plant time == wall time ==
# the FC's own clock without jitter feeding the AHRS. 1 kHz coupling --
# requires the Linux container SITL (~0.5 ms MSP roundtrip); the Windows
# cygwin SITL.exe is capped at ~64 Hz by the 15.6 ms timer tick.
DT = 0.001
_late = [0]        # slots we failed to hold (diagnostic)
_prof = {"msp": 0.0, "js": 0.0, "n": 0}
_step_clock = [0.0]   # wall-clock of the last plant step (see loop below)

_frames = [0]      # injected frames = sim time in ms (lockstep time base)

# Human throttle: the show formats script each phase with a FIXED stick
# value, and the hard step at every phase boundary reads as the thrust
# flicking back and forth in the replay (Daniel). A pilot moves the stick
# over ~a second - slew the COMMANDED throttle at full-travel-per-second.
# Only for the narrative formats; the special tests (crash gesture, floor
# panic chop) rely on crisp steps.
_THR_SLEW_US_PER_S = 1000.0
_THR_SLEW_ON = _man in ("show", "gyro_tip")
_thr_sent = [None]

def loop(secs, phase, rc, thr_override=None, print_every=1.0, freeze=False, gps=None):
    """freeze=True: hold the plant motionless (clean static IC while the FC
    settles/calibrates/arms disarmed) -- sensors still stream."""
    last = 0.0
    t0 = time.time()
    f0 = _frames[0]
    # lockstep: phase durations count SIM time (frames), the run may be
    # faster or slower than the wall clock
    while (_frames[0] - f0 < secs / DT) if LOCKSTEP else (time.time() - t0 < secs):
        it0 = time.perf_counter()
        # NOTE: rc stays the CALLER'S target - the slewed value lives in
        # _thr_sent and is applied on a copy (rebinding rc made the slew
        # chase its own output and freeze the stick, measured)
        rc_sent = rc
        if _THR_SLEW_ON and not freeze:
            if _thr_sent[0] is None:
                _thr_sent[0] = float(rc[2])
            _step = _THR_SLEW_US_PER_S * DT
            _thr_sent[0] += max(-_step, min(_step, rc[2] - _thr_sent[0]))
            rc_sent = rc[:2] + [int(round(_thr_sent[0]))] + rc[3:]
        # NOTE: no GPS injection for now. Injecting our GPS (even only while
        # upright) biases the AHRS pitch via the COG/vel fusion -- revisit
        # together with the lock-quality-gated altitude-source feature.
        r = sim_step(m, plant.acc_mg(), plant.gyro_dps16(), rc_sent, baro_pa=plant.baro_pa(), gps=gps)
        if r.debug[0] == 7:            # FW safety word cycles in slot 7
            _safety_cache[0] = r.debug[1]
        t_msp = time.perf_counter()
        ail = -r.stab_roll if FLIP_AIL else r.stab_roll
        ele = -r.stab_pitch if FLIP_ELE else r.stab_pitch
        rud = -r.stab_yaw if FLIP_RUD else r.stab_yaw
        thr = thr_override if thr_override is not None else (r.stab_throttle + 1.0) / 2.0
        _g = tvc_gain(thr)                       # firmware TVC servo commands
        tvcp = max(-1.0, min(1.0, r.stab_pitch * _g))
        tvcy = max(-1.0, min(1.0, r.stab_yaw * _g))
        if freeze:
            plant._a_earth = (0.0, 0.0, 0.0)   # static: pure gravity, zero rates
            _step_clock[0] = time.perf_counter()
        else:
            plant.set_controls(ail, ele, rud, thr)
            if _man == "hang_tvc" or _model == "funjet":
                # TVC airframe: the vectored nozzle works in EVERY flight
                # (show sequence included), not only the standalone demo
                plant.set_tvc(tvcp, tvcy)
            if LOCKSTEP:
                # the FC clock advances exactly DT per frame - fixed step,
                # perfectly equidistant, host load cannot matter
                plant.step(dt=DT)
            else:
                # WALL-CLOCK stepping, not a fixed DT: the FC's AHRS runs on
                # its own real-time clock and keeps integrating the LAST
                # injected gyro through any slot overrun. A fixed-step plant
                # freezes in that gap and the estimate runs 20+ deg ahead of
                # the truth during a fast entry - the aircraft then creeps to
                # the real attitude at the slow acc-correction rate (~2 deg/s,
                # seen as 156 -> 180 over 16 s). Stepping the plant by the
                # measured elapsed time keeps both integrations consistent;
                # rates change little within a gap, so the residual error is
                # second order.
                _now = time.perf_counter()
                _dt = min(max(_now - _step_clock[0], 0.0005), 0.05)
                _step_clock[0] = _now
                plant.step(dt=_dt)
        t_js = time.perf_counter()
        _prof["msp"] += t_msp - it0; _prof["js"] += t_js - t_msp; _prof["n"] += 1
        jr, jp, jy = plant.rpy()
        fr, fp, fy = r.att_roll_deg, r.att_pitch_deg, r.att_yaw_deg   # aus der Reply -- keine Extra-Roundtrips
        _frames[0] += 1
        t = _frames[0] * DT if LOCKSTEP else time.time() - T0
        if t - _mode_cache[1] > 0.1:             # poll real FC mode + baro alt + GPS at ~10 Hz
            _mode_cache[0] = fc_mode(m, BOXIDS)
            _alt_cache[0] = fc_alt_m(m)
            try:                                 # MSP_RAW_GPS: fixType u8, numSat u8
                _g = m.request(106)
                _gps_cache[0], _gps_cache[1] = _g[0], _g[1]
            except Exception:
                pass
            # track the WORST FC-vs-truth divergence, not just the final one:
            # a mid-flight AHRS offset (seen at 10 deg during an inverted
            # hold) is invisible in an end-of-flight check. Skip the frozen
            # settle/cal phases, the AHRS is converging there by design.
            if phase not in ("settle", "cal", "armL", "armH"):
                _d = _tilt_div_deg(fr, fp, jr, jp)
                if _d > _div_max[0]:
                    _div_max[0], _div_max[1], _div_max[2] = _d, t, phase
            _mode_cache[1] = t
        mode = _mode_cache[0]
        fc_thr = (r.stab_throttle + 1.0) / 2.0   # FC's own throttle output, even when overridden
        log.write(f"{t:.2f},{phase},{mode},{fr:.1f},{fp:.1f},{fy:.0f},"
                  f"{jr:.1f},{jp:.1f},{jy:.1f},{plant.ias_kts():.0f},{plant.z:.1f},"
                  f"{ail:.2f},{ele:.2f},{rud:.2f},{thr:.2f},{fc_thr:.2f},"
                  f"{rc_sent[0]},{rc_sent[1]},{rc_sent[2]},{rc_sent[3]},{rc_sent[4]},{rc_sent[5]},{rc_sent[6]},{rc_sent[7]},"
                  f"{_alt_cache[0]:.1f},{tvcp:.2f},{tvcy:.2f},{plant.xy()[0]:.1f},{plant.xy()[1]:.1f},"
                  f"{_gps_cache[0]},{_gps_cache[1]},{getattr(plant, '_flap_pos', 0.0):.2f},"
                  f"{_safety_cache[0]}\n")
        if time.time() - last > print_every:
            print(f"  [{phase:7}] FC {fr:+7.1f}/{fp:+6.1f}/{fy:3.0f} | JS {jr:+7.1f}/{jp:+6.1f}/{jy:5.1f} | "
                  f"IAS {plant.ias_kts():3.0f} alt {plant.z:5.0f} ele {ele:+.2f}")
            last = time.time()
        # hold the fixed slot: coarse sleep, then spin the last ~2 ms
        # (Windows sleep granularity would otherwise blow the slot).
        # Lockstep needs no pacing: the FC clock only moves with our frames.
        if not LOCKSTEP:
            while True:
                rem = DT - (time.perf_counter() - it0)
                if rem <= 0:
                    break
                if rem > 0.002:
                    time.sleep(rem - 0.002)
            if -(DT - (time.perf_counter() - it0)) > 0.005:
                _late[0] += 1                    # slot overran by >5 ms

print("boot-cal abwarten (bench-Routine)...")
from bench import wait_boot_calibration
wait_boot_calibration(m)

print("=== SETTLE (align the AHRS to JSBSim, plant frozen) ===")
loop(6, "settle", rc_ch(), freeze=True)
fr, fp, fy = fc_att(m); jr, jp, jy = plant.rpy()
print(f"Konventions-Check: FC {fr:+.1f}/{fp:+.1f} vs JS {jr:+.1f}/{jp:+.1f}  "
      f"({'OK' if abs(fr-jr)<15 and abs(fp-jp)<15 else 'MISMATCH -> Vorzeichen pruefen'})")

print("=== CAL (run the HITL stream until the cal blockers clear) ===")
# bit9 sensors calibrating, bit13 acc-not-calibrated: the HITL enable path
# sets ACCELEROMETER_CALIBRATED on the first sim frame, but a boot race can
# leave bit13 up - keep streaming until BOTH are gone
_CAL_BLOCK = FLAG_CAL | (1 << 13)
t0 = time.time()
while (arming_flags(m) & _CAL_BLOCK) and time.time() - t0 < 40:
    loop(1.0, "cal", rc_ch(angle=RC_HIGH), print_every=4, freeze=True)
print("cal fertig, flags=0x%X" % arming_flags(m))

print("=== ARM (toggle cycles until ARMED) ===")
t0 = time.time()
while not (arming_flags(m) & FLAG_ARMED) and time.time() - t0 < 30:
    loop(1.0, "armL", rc_ch(thr=RC_LOW, arm=RC_LOW, angle=RC_HIGH), print_every=9, freeze=True)
    loop(1.2, "armH", rc_ch(thr=RC_LOW, arm=RC_HIGH, angle=RC_HIGH), print_every=9, freeze=True)
print("ARMED:", bool(arming_flags(m) & FLAG_ARMED), f"flags=0x{arming_flags(m):X}")
if not (arming_flags(m) & FLAG_ARMED):
    # NEVER fly dead: an unarmed FC outputs zero throttle and the whole
    # flight glides silently into the ground (measured) - abort loudly so
    # the runner can restart the SITL and retry
    log.close()
    raise SystemExit(f"ABORT: FC wuerde nicht armen, flags=0x{arming_flags(m):X}")

# released from the frozen IC only now: level + settle for a few seconds so
# the whole loop (plant, AHRS, controller) is in steady state before testing
print("=== ANGLE LEVEL (settle from a clean initial condition) ===")
loop(6, "level", rc_ch(thr=1650, arm=RC_HIGH, angle=RC_HIGH))   # 1450 = level trim (~50 kts, 0 m/min)

MAN = _man
MAN_RC = {   # SEL detents: 1270 INVERT / 1510 KN L / 1750 KN R / 1985 HANG
    "inverted":    dict(sel=1270),
    "inverted_stick": dict(sel=1270),   # stick-offset carving around inverted
    "knife_left":  dict(sel=1510),
    "knife_right": dict(sel=1750),
    "hang":        dict(sel=1985),
    "roll_hold":   dict(angle=1525),                  # FIGROLL band on the mode selector
    "loop_fig":    dict(angle=1225),                  # FIGLOOP band on the mode selector
    "floor_dive":  dict(angle=RC_HIGH, floor=1900),   # ANGLE + FLOOR switch on
    "floor_panic": dict(angle=RC_HIGH, floor=1900),   # dive with the throttle CHOPPED
    "floor_spin":  dict(floor=1900),                  # flat spin INTO the floor (body sets mode)
    "crash_test":  dict(angle=RC_HIGH),               # impact + stillness -> motor cut + gesture
    "snap_neg":    dict(angle=RC_HIGH),               # impact + keeps flying -> must NOT cut
    "flat_spin":   dict(angle=1375),                  # FSPIN band on the mode selector (pilot rudder)
    "inv_spin":    dict(sel=1270, angle=1375),        # FSPIN + INVERTED: inverted flat spin
    "knife_spin":  dict(sel=1510, angle=1375),        # FSPIN + KNIFE L: knife edge spin
    "hang_tvc":    dict(sel=1985),                    # prop hang on the TVC pusher delta
    "seq":         dict(angle=1675),                  # FIGSEQ band: flies whatever
                                                      # sequence figure_script.py programmed
    "seq_chain":   dict(angle=1675),                  # three routines back-to-back,
                                                      # reprogrammed via MSP between legs
    "show":        dict(angle=RC_HIGH),               # one video per airplane:
                                                      # capability-derived sequence
    "gyro_tip":    dict(angle=RC_HIGH),               # autogyro tip-over pair:
                                                      # --guard adds ROTOR GUARD
}[MAN]
thrM = 1500 if MAN in ("hang", "hang_tvc") else 1650   # level trim; holds start stable (hang: hover PID owns)
# --thr <us>: maneuver-throttle override for airframes whose power differs
# from the aerobat3d the default bands were trimmed on (a T/W-2 turbotimber
# climbs out of the video ceiling at 1650)
if "--thr" in sys.argv:
    thrM = int(sys.argv[sys.argv.index("--thr") + 1])


# --- MANUAL: pilot flies by hand in ANGLE so the sticks visibly move,
#     then we flip the figure switch -> the sequence takes over.
#     The show format skips this (Daniel: no manual intro). ---
if MAN != "show":
    print("=== MANUAL (pilot flies by hand in ANGLE, sticks visibly move) ===")
    loop(3, "manual", rc_ch(thr=1650, arm=RC_HIGH, angle=RC_HIGH, ail=1250))
    loop(3, "manual", rc_ch(thr=1650, arm=RC_HIGH, angle=RC_HIGH, ail=1750))
    loop(3, "manual", rc_ch(thr=1650, arm=RC_HIGH, angle=RC_HIGH))

if FLOOR_ON and not MAN.startswith("floor") and plant.z < _CLIMB_TARGET_M - 5:
    # scripted climb to the maneuver altitude; the floor arms on the way
    print(f"=== CLIMB to {_CLIMB_TARGET_M:.0f} m (floor arms passing {30}+margin) ===")
    _climb_t = 0
    while plant.z < _CLIMB_TARGET_M - 3 and _climb_t < 90:
        loop(1, "climb", rc_ch(thr=1900, arm=RC_HIGH, ele=1800, angle=RC_HIGH), print_every=1)
        _climb_t += 1

print(f"=== SEQUENCE {MAN} (handover manual -> controller) ===")
if MAN == "floor_dive":
    # climb in ANGLE (no floor yet), flip the FLOOR switch on well ABOVE the
    # line, then push over and HOLD down-elevator -- the floor must catch and
    # level AGAINST the held stick, not because we let go. Then a contrast
    # pass with the FLOOR switch OFF: the same held push punches through.
    loop(6, "climb", rc_ch(thr=1900, arm=RC_HIGH, ele=1800, angle=RC_HIGH), print_every=1)
    loop(2, "arm-floor", rc_ch(thr=1700, arm=RC_HIGH, angle=RC_HIGH, floor=1900), print_every=1)
    loop(10, "dive-held", rc_ch(thr=1700, arm=RC_HIGH, ele=1150, angle=RC_HIGH, floor=1900), print_every=0.7)
    loop(4, "climb2", rc_ch(thr=1900, arm=RC_HIGH, ele=1800, angle=RC_HIGH, floor=1900), print_every=1)
    loop(6, "dive-nofloor", rc_ch(thr=1700, arm=RC_HIGH, ele=1150, angle=RC_HIGH), print_every=0.7)
elif MAN in ("hang", "hang_tvc"):
    loop(6, MAN, rc_ch(thr=thrM, arm=RC_HIGH, angle=RC_LOW, **MAN_RC), print_every=0.7)
    plant.set_wind(down_ms=3.0)
    loop(4, "gust", rc_ch(thr=thrM, arm=RC_HIGH, angle=RC_LOW, **MAN_RC), print_every=0.7)
    plant.set_wind()
    loop(8, MAN, rc_ch(thr=thrM, arm=RC_HIGH, angle=RC_LOW, **MAN_RC), print_every=0.7)
    # exit transition: drop the target, ANGLE catches it back to level flight
    loop(8, "exit", rc_ch(thr=1650, arm=RC_HIGH, angle=RC_HIGH), print_every=0.7)
elif MAN == "loop_fig":
    # full loop at fig_loop_rate, then level hold with assist; full power
    # through the figure so the top has energy
    loop(16, "loop", rc_ch(thr=1900, arm=RC_HIGH, **MAN_RC), print_every=0.7)
    loop(6, "level", rc_ch(thr=1650, arm=RC_HIGH, **MAN_RC), print_every=0.7)
    loop(4, "exit", rc_ch(thr=1650, arm=RC_HIGH, angle=RC_HIGH), print_every=0.7)
elif MAN in ("flat_spin", "inv_spin", "knife_spin"):
    # FLAT SPIN family: the box holds the selected attitude (flat, inverted
    # or knife edge via SEL) while the pilot's rudder commands the rotation
    # about the earth vertical (idle throttle, full rudder); releasing the
    # rudder stops the rotation with the attitude still held, releasing the
    # box recovers to ANGLE
    loop(4, "spin-hold", rc_ch(thr=1650, arm=RC_HIGH, **MAN_RC), print_every=0.7)
    loop(10, "spin-rud", rc_ch(thr=1000, arm=RC_HIGH, rud=2000, **MAN_RC), print_every=0.7)
    loop(5, "rud-release", rc_ch(thr=1650, arm=RC_HIGH, **MAN_RC), print_every=0.7)
    loop(5, "exit", rc_ch(thr=1650, arm=RC_HIGH, angle=RC_HIGH), print_every=0.7)
elif MAN == "floor_spin":
    # FLAT SPIN into the floor. FLOOR lives on its OWN switch (floor=1900 on
    # CH_INVERTED) and is armed once during the climb, then LEFT ON the whole
    # time - it never moves. The flight-mode selector flips ANGLE -> FSPIN for
    # the spin (angle 1375) and back to ANGLE on exit. No mode range is ever
    # remapped in flight; the floor must catch the autorotation on its own,
    # with the pilot at idle throttle and no elevator.
    loop(5, "climb", rc_ch(thr=1900, arm=RC_HIGH, ele=1800, angle=RC_HIGH), print_every=1)
    loop(4, "arm-floor", rc_ch(thr=1900, arm=RC_HIGH, ele=1800, angle=RC_HIGH, floor=1900), print_every=1)
    loop(3, "spin-hold", rc_ch(thr=1650, arm=RC_HIGH, angle=1375, floor=1900), print_every=0.7)
    loop(8, "flat-spin", rc_ch(thr=1000, arm=RC_HIGH, rud=2000, angle=1375, floor=1900), print_every=0.7)
    loop(12, "caught", rc_ch(thr=1050, arm=RC_HIGH, angle=1375, floor=1900), print_every=0.7)
    loop(5, "exit", rc_ch(thr=1650, arm=RC_HIGH, angle=RC_HIGH, floor=1900), print_every=0.7)
elif MAN == "floor_panic":
    # dive with the throttle CHOPPED (the panic case): the recovery climb
    # must get its own energy (cruise + pitch-to-throttle floor), the motor
    # must keep RUNNING through the low stick, and the held down-elevator
    # must not drag the recovery target down (stick rates suppressed)
    loop(8, "climb", rc_ch(thr=1900, arm=RC_HIGH, ele=1800, angle=RC_HIGH), print_every=1)
    loop(2, "arm-floor", rc_ch(thr=1700, arm=RC_HIGH, angle=RC_HIGH, floor=1900), print_every=1)
    loop(20, "dive-chop", rc_ch(thr=1050, arm=RC_HIGH, ele=1150, angle=RC_HIGH, floor=1900), print_every=0.7)
    loop(8, "after", rc_ch(thr=1050, arm=RC_HIGH, angle=RC_HIGH, floor=1900), print_every=0.7)
    # pilot takeover: after the catch (sticks were centered in "after") a
    # fresh deflection must end the recovery immediately - ANGLE is back
    loop(6, "takeover", rc_ch(thr=1650, arm=RC_HIGH, ele=1400, angle=RC_HIGH, floor=1900), print_every=0.7)
elif MAN == "crash_test":
    # crash detection POSITIVE path: impact spike, then the airframe lies
    # still (frozen plant = frozen baro, 1 g, zero rates, GPS speed 0) ->
    # motor cut ~1.5 s later; throttle low-then-up gesture re-allows it
    GPS_STILL = dict(lat_e7=473970000, lon_e7=85400000, alt_cm=12000,
                     speed_cms=0, course_dd=0, vel_ned_cms=(0, 0, 0))
    loop(5, "cruise", rc_ch(thr=1650, arm=RC_HIGH, angle=RC_HIGH), print_every=1, gps=GPS_STILL)
    for _ in range(3):     # realistic impact: a 3 ms spike pegged at full-scale
        sim_step(m, (0.0, 0.0, 16000.0), plant.gyro_dps16(),   # then NOTHING (still)
                 rc_ch(thr=1650, arm=RC_HIGH, angle=RC_HIGH), baro_pa=plant.baro_pa(), gps=GPS_STILL)
        _frames[0] += 1
    loop(4, "still", rc_ch(thr=1650, arm=RC_HIGH, angle=RC_HIGH), print_every=0.5, freeze=True, gps=GPS_STILL)
    loop(1.5, "ack-low", rc_ch(thr=1000, arm=RC_HIGH, angle=RC_HIGH), print_every=0.5, freeze=True, gps=GPS_STILL)
    loop(3, "re-up", rc_ch(thr=1400, arm=RC_HIGH, angle=RC_HIGH), print_every=0.5, freeze=True, gps=GPS_STILL)
elif MAN == "snap_neg":
    # crash detection NEGATIVE path: the same spike, but the aircraft keeps
    # flying - including the hard case of a smooth level line right after
    # the pull, which only the GPS ground speed can tell from lying still
    GPS_MOVE = dict(lat_e7=473970000, lon_e7=85400000, alt_cm=12000,
                    speed_cms=1800, course_dd=0, vel_ned_cms=(1800, 0, 0))
    loop(5, "cruise", rc_ch(thr=1650, arm=RC_HIGH, angle=RC_HIGH), print_every=1, gps=GPS_MOVE)
    for _ in range(3):     # same 3 ms full-scale spike, but the aircraft keeps
        sim_step(m, (0.0, 0.0, 16000.0), plant.gyro_dps16(),   # moving -> must NOT cut
                 rc_ch(thr=1650, arm=RC_HIGH, angle=RC_HIGH), baro_pa=plant.baro_pa(), gps=GPS_MOVE)
        _frames[0] += 1
    loop(5, "flyon", rc_ch(thr=1650, arm=RC_HIGH, angle=RC_HIGH, ail=1800), print_every=0.5, gps=GPS_MOVE)
    loop(6, "flyon2", rc_ch(thr=1650, arm=RC_HIGH, angle=RC_HIGH, ail=1300), print_every=0.5, gps=GPS_MOVE)
elif MAN == "seq":
    # fly whatever sequence figure_script.py programmed (video pipeline):
    # full power through the figures, the sequencer owns the trajectory
    loop(90, "seq", rc_ch(thr=1800, arm=RC_HIGH, **MAN_RC), print_every=0.7)
    loop(6, "exit", rc_ch(thr=1650, arm=RC_HIGH, angle=RC_HIGH), print_every=0.7)
elif MAN == "seq_chain":
    # three routines in ONE flight: the runner reprograms the sequence via
    # MSP between the legs (the segment setter is live, no eeprom save) and
    # re-enters F SEQ for each - the box edge restarts at segment 0
    import json as _json
    from figure_script import compile_script as _compile, MAX_SEGMENTS as _MAXSEG, FIGSEG_END as _END
    CHAIN = [("examples/veloxity_3d_demo.json", 58),
             ("examples/wargo_vol8_immelmann_inverted.json", 26),
             ("examples/wargo_gyro_knife_pass.json", 26)]
    for _path, _secs in CHAIN:
        _c = _compile(_json.load(open(_path, encoding="utf-8")))
        for _i, _seg in enumerate(_c):
            m.set_figure_segment(_i, *_seg)
        for _i in range(len(_c), _MAXSEG):
            m.set_figure_segment(_i, _END)
        _leg = _path.rsplit("/", 1)[-1].removesuffix(".json")
        loop(_secs, f"seq {_leg}", rc_ch(thr=1800, arm=RC_HIGH, angle=1525), print_every=0.7)
        loop(4, "between", rc_ch(thr=1650, arm=RC_HIGH, angle=RC_HIGH), print_every=1)
    loop(3, "exit", rc_ch(thr=1650, arm=RC_HIGH, angle=RC_HIGH), print_every=0.7)
elif MAN == "show":
    # ONE VIDEO PER AIRPLANE: the sequence derives from the actuator-true
    # repertoire in airframe_config.py - no maneuver an airframe obviously
    # cannot fly. No manual intro; a horizontal base line before every
    # figure; the FLOOR switch stays on the whole flight (armed during the
    # climbs). Opens with the Einflug: the level throttle is found by
    # feedback and recorded per model in airframe_trim.json.
    import json as _json
    import os as _os
    from airframe_config import AIRFRAMES as _AF
    _act, _rep = _AF[_model]
    if _act == "GYRO":
        raise SystemExit("the gyro flies its own tip-over pair, not `show`")

    print(f"=== EINFLUG {_model} (find the level throttle) ===")
    _thrL, _z0 = 1550.0, plant.z
    for _ in range(20):
        loop(0.5, "einflug", rc_ch(thr=int(_thrL), arm=RC_HIGH, angle=RC_HIGH),
             print_every=2)
        _vz = (plant.z - _z0) / 0.5
        _z0 = plant.z
        _thrL = min(1900.0, max(1100.0, _thrL - 18.0 * _vz))
    thrL = int(_thrL)
    print(f"Einflug: level throttle {thrL}")
    _tf = (_json.load(open("airframe_trim.json"))
           if _os.path.exists("airframe_trim.json") else {})
    _tf[_model] = dict(thr_level=thrL)
    with open("airframe_trim.json", "w") as _fh:
        _json.dump(_tf, _fh, indent=1)
    with open(f"jsbsim_params_{_man}.txt", "a") as _pf:
        _pf.write(f"model={_model}\nthr_level={thrL}\n")

    def _to_alt(target, tmax=60, label="transit"):
        # transit to the figure's entry altitude BY ALTITUDE, not by time
        # (a slow climber gets as long as it needs - the PT-17 lesson),
        # then a horizontal base line at the trimmed throttle. The descent
        # is SPEED-AWARE: leaving a slow figure (harrier/hang exit) the
        # elevator has no authority - power up level first, only a flying
        # airframe can be pushed down (the mush-climb trap).
        _t0 = _frames[0]
        while abs(plant.z - target) > 4 and (_frames[0] - _t0) * DT < tmax:
            if plant.z < target - 6:
                # climb throttle relative to the trim, not pegged: an
                # overpowered EDF at full power accelerates so hard that
                # the accelerometer bends the AHRS gravity estimate
                # (measured: 17 deg pitch divergence on the lippisch)
                loop(0.7, label, rc_ch(thr=min(1900, thrL + 450), ele=1800,
                                           arm=RC_HIGH, angle=RC_HIGH), print_every=2)
            elif plant.z < target:
                # fine approach from below: gentle, no overshoot
                loop(0.7, label, rc_ch(thr=min(1900, thrL + 150), ele=1650,
                                           arm=RC_HIGH, angle=RC_HIGH), print_every=2)
            elif plant.ias_kts() < 22:
                loop(0.7, label, rc_ch(thr=min(1900, thrL + 200), arm=RC_HIGH,
                                           angle=RC_HIGH), print_every=2)
            else:
                loop(0.7, label, rc_ch(thr=max(1100, thrL - 200), ele=1300,
                                           arm=RC_HIGH, angle=RC_HIGH), print_every=2)
        loop(3, label.replace("transit", "base"), rc_ch(thr=thrL, arm=RC_HIGH, angle=RC_HIGH), print_every=2)

    def _exit():
        # energy-aware exit: a fast figure ends with excess speed, and at
        # the trimmed throttle ANGLE converts that straight into a zoom
        # over the 122 m ceiling (measured: 71 kts -> 146 m). Bleed at
        # idle with the nose SLIGHTLY down - level-pitch bleeding still
        # zooms (+12 m measured), only drag on a shallow downline kills
        # the energy without buying height.
        loop(4, "bleed", rc_ch(thr=1100, ele=1400, arm=RC_HIGH, angle=RC_HIGH),
             print_every=0.7)
        loop(2, "exit", rc_ch(thr=thrL, arm=RC_HIGH, angle=RC_HIGH), print_every=0.7)

    def _fig_inverted():
        _to_alt(95)
        loop(8, "inverted", rc_ch(thr=thrL, arm=RC_HIGH, angle=RC_LOW, sel=1270),
             print_every=0.7)
        # bleed INSIDE the hold before releasing it: dropping the box at
        # full trim speed makes a hot delta snap ~150 deg of roll at
        # something like 3000 deg/s, and the attitude estimate disagrees by
        # ~28 deg mid-snap (measured, funjet). Slow down first, release
        # second - half the energy, a civilized roll-out, honest AHRS.
        loop(3, "inv-bleed", rc_ch(thr=1100, arm=RC_HIGH, angle=RC_LOW, sel=1270),
             print_every=0.7)
        loop(3, "exit", rc_ch(thr=thrL, arm=RC_HIGH, angle=RC_HIGH), print_every=0.7)

    def _fig_roll():
        _to_alt(95)
        loop(8, "roll", rc_ch(thr=min(1900, thrL + 250), arm=RC_HIGH, angle=1525),
             print_every=0.7)
        _exit()

    def _fig_loop():
        # ONE loop (fig_loop_rate 90 deg/s = 4 s) plus margin; throttle
        # relative to the trim - a T/W-2 airframe at full power hits
        # 100 kts in the downline and the exit zooms out of the ceiling
        # 78 m entry: a hot delta pulls the loop BOTTOM 19 m below the
        # entry (measured on the lippisch, 70 -> 51) and genuinely breaks
        # the 55 m floor line - entry 78 keeps the deepest measured bottom
        # above it, and trim+250 caps the strongest climber's top under
        # the 122 ceiling (~40 m gain)
        _to_alt(78)
        # the loop apex is entry + 2v/omega: at the 90 deg/s figure rate
        # only entries below ~62 kts fit under the 122 ceiling from 78 m
        # (a 74 kt jet peaked 127, measured) - decelerate first, the loop
        # itself then needs only a modest energy margin
        _t0d = _frames[0]
        while plant.ias_kts() > 62 and (_frames[0] - _t0d) * DT < 12:
            loop(0.5, "base", rc_ch(thr=1150, arm=RC_HIGH, angle=RC_HIGH),
                 print_every=2)
        loop(6, "loop", rc_ch(thr=min(1900, thrL + 150), arm=RC_HIGH, angle=1225),
             print_every=0.7)
        _exit()

    def _fig_knife(fast=False):
        # knife edge needs speed: the fuselage carries the weight on the
        # rudder - but only modest margin over the trim, an overpowered
        # airframe otherwise accelerates through the hold (ias 70 measured
        # at trim+150) and the exit energy blows the ceiling
        _thr = min(1900, thrL + (300 if fast else 100))
        _to_alt(85)
        loop(6, "knife-L", rc_ch(thr=_thr, arm=RC_HIGH, angle=RC_LOW, sel=1510),
             print_every=0.7)
        loop(3, "base", rc_ch(thr=thrL, arm=RC_HIGH, angle=RC_HIGH), print_every=0.7)
        loop(6, "knife-R", rc_ch(thr=_thr, arm=RC_HIGH, angle=RC_LOW, sel=1750),
             print_every=0.7)
        _exit()

    def _fig_spin():
        # the spin eats 60-75 m and must COMPLETE above the floor line
        # (55 m true) so the figure is its own proof, not floor-assisted
        # (measured: entry 100 flew 52 percent of the spin under floor
        # override). Daniel: go higher if it needs the height - the spin
        # segment may exceed the 122 m video ceiling, the gate scopes it.
        _to_alt(130, label="transit-spin")
        loop(3, "spin-hold", rc_ch(thr=thrL, arm=RC_HIGH, angle=1375), print_every=0.7)
        loop(5, "spin-rud", rc_ch(thr=1000, rud=2000, arm=RC_HIGH, angle=1375),
             print_every=0.7)
        # NO bleed after a spin: it ends SLOW by design - there is no excess
        # speed to dissipate, an idle nose-down bleed only eats the altitude
        # margin the recovery just saved (measured: bf109 down to 12.9 m)
        loop(4, "rud-release", rc_ch(thr=thrL, arm=RC_HIGH, angle=1375), print_every=0.7)
        loop(3, "exit", rc_ch(thr=thrL, arm=RC_HIGH, angle=RC_HIGH), print_every=0.7)

    def _fig_hang():
        # the pull converts the base-line speed to height (v^2/2g plus the
        # hover-PID settle) - enter low so the hang sits mid-window; the
        # exit dive rebuilds speed, so it too bleeds at idle first
        _to_alt(65)
        loop(10, "hang", rc_ch(thr=1500, arm=RC_HIGH, angle=RC_LOW, sel=1985),
             print_every=0.7)
        _exit()

    def _fig_flaps_harrier():
        # slow flaps out (2 s servo travel), high alpha, reduced power -
        # the blown-flap harrier pass
        _to_alt(72)
        plant.set_flaps(1.0)
        loop(3, "flaps-out", rc_ch(thr=thrL, arm=RC_HIGH, angle=RC_HIGH), print_every=0.7)
        loop(10, "harrier", rc_ch(thr=max(1100, thrL - 150), ele=1900, arm=RC_HIGH,
                                  angle=RC_HIGH), print_every=0.7)
        plant.set_flaps(0.0)
        loop(4, "exit", rc_ch(thr=thrL, arm=RC_HIGH, angle=RC_HIGH), print_every=0.7)

    def _fig_flaps_slow():
        # full flaps, reduced power: the slow pass (a10/binary class)
        _to_alt(70)
        plant.set_flaps(1.0)
        loop(8, "flaps-slow", rc_ch(thr=max(1100, thrL - 120), arm=RC_HIGH,
                                    angle=RC_HIGH), print_every=0.7)
        plant.set_flaps(0.0)
        loop(4, "exit", rc_ch(thr=thrL, arm=RC_HIGH, angle=RC_HIGH), print_every=0.7)

    _FIGS = {"inverted": _fig_inverted, "roll": _fig_roll, "loop": _fig_loop,
             "knife": _fig_knife, "knife_fast": lambda: _fig_knife(fast=True),
             "spin": _fig_spin, "hang": _fig_hang,
             "flaps_harrier": _fig_flaps_harrier, "flaps_slow": _fig_flaps_slow}
    for _r in _rep:
        print(f"=== SHOW {_model}: {_r} ===")
        _FIGS[_r]()
    # THE FINALE (Daniel): one explicit floor test closes every show -
    # dive into the net with the elevator HELD, the floor must catch and
    # climb out against the stick. The figures above never needed to skim
    # the line; this single catch is the floor's proof.
    print(f"=== SHOW {_model}: floor test ===")
    # from 110: the predictive catch fires around 85 true regardless (3 s
    # lookahead), but the dive ahead of it is VISIBLE - from 75 the whole
    # finale played out in 11 m (measured), no demo at all
    _to_alt(110, label="transit-floor")
    # TRIM-RELATIVE throttles (fixed values were the timber lesson again:
    # 1700 on a T/W-2 airframe is fast level flight, not a dive, and a
    # 1650 climb-out rockets to 161 m): near-idle dive so EVERY airframe
    # genuinely descends, trim throttle after the catch so the recovery
    # levels off instead of zooming
    # dive UNTIL the catch (FW safety word), not for a fixed time: a
    # floaty glider sinks 4 m/s and needs 15+ s to reach the engage
    # envelope, an overpowered trainer is there in 4 (measured both ways);
    # the 25 s timeout leaves a missing catch for the gate to fail
    _t0f = _frames[0]
    while not (_safety_cache[0] & 2) and (_frames[0] - _t0f) * DT < 25:
        loop(0.5, "floor-dive", rc_ch(thr=max(1100, thrL - 150), ele=1150,
                                      arm=RC_HIGH, angle=RC_HIGH), print_every=2)
    loop(6, "caught", rc_ch(thr=thrL, arm=RC_HIGH, angle=RC_HIGH), print_every=0.7)
elif MAN == "gyro_tip":
    # THE TIP-OVER PAIR (floor_dive contrast pattern, Daniel's spec): slow
    # flight starves the rotor - rpm decays with the inflow, the lateral
    # tilt goes soft (authority ~ rpm^2), the blade-asymmetry left pull
    # wins and the gyro rolls away with the stick at the stop. Sequence 1
    # (default) flies WITHOUT protection: the honest failure, in it goes.
    # Sequence 2 (--guard) has the ROTOR GUARD box on: the FW catches the
    # excursion - wings level, nose down, throttle floor - because thrust
    # is the only lever that restores inflow -> rpm -> authority.
    GUARD = "--guard" in sys.argv
    _sel = 1900 if GUARD else RC_LOW
    with open(f"jsbsim_params_{_man}.txt", "a") as _pf:
        _pf.write(f"model={_model}\n")
    print(f"=== GYRO TIP ({'WITH' if GUARD else 'WITHOUT'} rotor guard) ===")
    # healthy entry: enough throttle that the rotor sits ABOVE the stall
    # band before the starving begins - the story arc needs a clean start
    loop(6, "cruise", rc_ch(thr=1700, arm=RC_HIGH, angle=RC_HIGH, sel=_sel),
         print_every=0.7)
    # starve the rotor: near-idle, ANGLE holds level while the speed and
    # with it the rotor rpm bleed away - the tip-over regime from the
    # research (and the plant's measured rpm decay)
    loop(14, "slow-decay", rc_ch(thr=1120, arm=RC_HIGH, angle=RC_HIGH, sel=_sel),
         print_every=0.7)
    loop(5, "tip-window", rc_ch(thr=1120, arm=RC_HIGH, angle=RC_HIGH, sel=_sel),
         print_every=0.7)
    # aftermath: without the guard this is wreckage by now; with it the
    # guard has caught every excursion (twice from 80 m) and the pilot
    # giving the throttle back gets a FLYING aircraft - 1700 is the
    # cruise value that holds level, the story must end airborne
    loop(10, "after", rc_ch(thr=1700, arm=RC_HIGH, angle=RC_HIGH, sel=_sel),
         print_every=0.7)
elif MAN == "inverted_stick":
    # ANGLE-semantics stick offsets: half aileron must carve a HELD angle
    # offset from the inverted reference (not a rate), releasing returns
    # the target gently; then the same on the elevator
    loop(8, "inverted", rc_ch(thr=thrM, arm=RC_HIGH, angle=RC_LOW, **MAN_RC), print_every=0.7)
    loop(4, "stick-roll", rc_ch(thr=thrM, arm=RC_HIGH, angle=RC_LOW, ail=1750, **MAN_RC), print_every=0.7)
    loop(4, "release", rc_ch(thr=thrM, arm=RC_HIGH, angle=RC_LOW, **MAN_RC), print_every=0.7)
    loop(4, "stick-pitch", rc_ch(thr=thrM, arm=RC_HIGH, angle=RC_LOW, ele=1250, **MAN_RC), print_every=0.7)
    loop(4, "release", rc_ch(thr=thrM, arm=RC_HIGH, angle=RC_LOW, **MAN_RC), print_every=0.7)
elif MAN == "inverted":
    # settle the hold straight first; the deliberate inverted turn comes
    # only once the hold is stable (pilot rudder, visible on the stick)
    loop(8, "inverted", rc_ch(thr=thrM, arm=RC_HIGH, angle=RC_LOW, **MAN_RC), print_every=0.7)
    plant.set_wind(down_ms=3.0)
    loop(4, "gust", rc_ch(thr=thrM, arm=RC_HIGH, angle=RC_LOW, **MAN_RC), print_every=0.7)
    plant.set_wind()
    loop(3, "inverted", rc_ch(thr=thrM, arm=RC_HIGH, angle=RC_LOW, **MAN_RC), print_every=0.7)
    loop(6, "inv-turn", rc_ch(thr=thrM, arm=RC_HIGH, angle=RC_LOW, rud=1650, **MAN_RC), print_every=0.7)
    loop(5, "inverted", rc_ch(thr=thrM, arm=RC_HIGH, angle=RC_LOW, **MAN_RC), print_every=0.7)
else:
    # hold with a disturbance: 4 s downdraft gust mid-hold -- the honest
    # proof of regulation is the visible actuator response and the altitude
    # error returning to zero afterwards. MAN_RC merges OVER the defaults so a
    # maneuver that drives the mode selector (roll_hold -> FIGROLL on angle)
    # overrides the ANGLE-off default instead of colliding with it.
    _hold_kw = {**dict(thr=thrM, arm=RC_HIGH, angle=RC_LOW), **MAN_RC}
    loop(8, MAN, rc_ch(**_hold_kw), print_every=0.7)
    plant.set_wind(down_ms=3.0)
    loop(4, "gust", rc_ch(**_hold_kw), print_every=0.7)
    plant.set_wind()
    loop(12, MAN, rc_ch(**_hold_kw), print_every=0.7)

fr, fp, fy = fc_att(m); jr, jp, jy = plant.rpy()
# per-maneuver end state: only the sustained holds END in the held attitude;
# figures/spins/exits end LEVEL by design (roll ~ 0)
_EXPECT = {
    "inverted": "|roll| ~ 180", "inverted_stick": "|roll| ~ 180",
    "knife_left": "roll ~ -90", "knife_right": "roll ~ +90",
    "hang": "pitch ~ +90 during hold, level after exit",
    "hang_tvc": "pitch ~ +90 during hold, level after exit",
}
print(f"FINAL: FC roll {fr:+.1f}  JS roll {jr:+.1f}  "
      f"(expected: {_EXPECT.get(MAN, 'level, roll ~ 0 - figure/spin ends level by design')})")
# FC-vs-truth divergence: a corrupted AHRS (sustained spin rotation) can
# report level while the plane spirals into the ground -- checking only
# the FC's own attitude waves that through. Compare the tilt of both.
def _up(r, p):
    r, p = math.radians(r), math.radians(p)
    return (-math.sin(p), math.sin(r) * math.cos(p), math.cos(r) * math.cos(p))
_uf, _uj = _up(fr, fp), _up(jr, jp)
_div = math.degrees(math.acos(max(-1.0, min(1.0, sum(a * b for a, b in zip(_uf, _uj))))))
print(f"FINAL: FC-vs-truth tilt divergence {_div:.1f} deg"
      + ("  << AHRS DIVERGED, estimate not trustworthy" if _div > 15 else ""))
print(f"FINAL: worst divergence {_div_max[0]:.1f} deg at t={_div_max[1]:.1f}s ({_div_max[2]})"
      + ("  << AHRS excursion mid-flight" if _div_max[0] > 15 else ""))
if _prof["n"]:
    print(f"timing: {_late[0]} slots overran >5ms | per cycle: "
          f"msp {1000*_prof['msp']/_prof['n']:.1f} ms, jsbsim {1000*_prof['js']/_prof['n']:.1f} ms "
          f"(slot {DT*1000:.0f} ms)")
log.close(); m.close()
