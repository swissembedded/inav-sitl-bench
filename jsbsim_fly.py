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
# channels: A E T R ARM ANGLE INVERT SELECT  (bench provisioning layout)
def rc_ch(thr=RC_LOW, arm=RC_LOW, angle=RC_LOW, invert=RC_LOW, sel=RC_LOW, ele=RC_MID, ail=RC_MID, rud=RC_MID):
    return [ail, ele, thr, rud, arm, angle, invert, sel]

# Real active flight mode, pulled from the FC itself (not re-derived from RC):
# MSP_BOXIDS gives permanent box ids in active order, MSP_ACTIVEBOXES gives a
# bit per active box in the same order. Orientation-hold sub-modes win over
# ANGLE. permanentId map from src/main/fc/fc_msp_box.c.
MSP_BOXIDS = 119
MSP_ACTIVEBOXES = 113
MSP_ALTITUDE = 109
PERM_NAME = {69: "INVERT", 70: "KNIFE L", 71: "KNIFE R", 72: "P-HANG",
             74: "F ROLL", 75: "F LOOP", 76: "F 4PT", 77: "F SEQ",
             79: "FLAT SPIN"}

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
    base = next((PERM_NAME[p] for p in (69, 70, 71, 72, 74, 75, 76, 77, 79)
                 if p in active), None)
    if base is None:
        base = "ANGLE" if 1 in active else ("ACRO" if 0 in active else "DISARMED")
    if 73 in active:                            # altitude floor engaged as suffix
        base += "+FLOOR"
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
_man = next((a for a in sys.argv[1:] if not a.startswith("--")), "inverted")
# default start = 120 m (legal RC ceiling); floor_dive trades ~100 m away,
# so it alone starts higher to leave room for the catch
_model = "c172p" if "--c172" in sys.argv else ("funjet" if _man == "tvc_hang" else "aerobat3d")
# 1500 ft for the diagnostic c172 (entry transient), 120 m for everything
# else -- the altitude floor works relative to the baro zero at start
plant = JSBSimPlant(model=_model,
                    alt_ft=1500 if _model == "c172p" else (820 if _man == "flat_spin" else 394))
log = open(f"jsbsim_log_{_man}.csv", "w")
log.write("t,phase,mode,fc_roll,fc_pitch,fc_yaw,js_roll,js_pitch,js_yaw,ias,alt,"
          "ail,ele,rud,thr,fc_thr,st_ail,st_ele,st_thr,st_rud,"
          "st_arm,st_angle,st_inv,st_sel,fc_alt,tvc_p,tvc_y,x,y,gps_fix,gps_sat\n")
BOXIDS = read_boxids(m)
_mode_cache = ["DISARMED", 0.0]     # [last mode string, last poll wall-time]
_alt_cache = [0.0]                  # last FC-measured altitude
_gps_cache = [0, 0]                 # [fixType, numSat] from MSP_RAW_GPS
_div_max = [0.0, 0.0, ""]           # [max FC-vs-truth tilt divergence, time, phase]

def _tilt_div_deg(fr, fp, jr, jp):
    def up(r, p):
        r, p = math.radians(r), math.radians(p)
        return (-math.sin(p), math.sin(r) * math.cos(p), math.cos(r) * math.cos(p))
    a, b = up(fr, fp), up(jr, jp)
    return math.degrees(math.acos(max(-1.0, min(1.0, sum(x * y for x, y in zip(a, b))))))
# --set name=value (repeatable): write firmware settings live before the
# flight, value size taken from the FC's SETTING_INFO reply (sweeps)
_argv = sys.argv[1:]
for _i, _a in enumerate(_argv):
    if _a == "--set" and _i + 1 < len(_argv) and "=" in _argv[_i + 1]:
        _n, _v = _argv[_i + 1].split("=", 1)
        _size = len(m.request(0x1003, _n.encode() + b"\x00"))
        m.request(0x1004, _n.encode() + b"\x00" + int(_v).to_bytes(_size, "little", signed=int(_v) < 0))
        print(f"set {_n} = {_v}")

PARAMS = ["fig_roll_rate", "fig_loop_rate", "fig_assist_z_gain", "fig_assist_vz_gain",
          "fig_assist_max", "ohold_inverted_pitch_trim", "ohold_knife_left_pitch_trim",
          "ohold_knife_right_pitch_trim", "ohold_hover_thr_p", "ohold_hover_thr_i",
          "ohold_hover_thr_d", "small_angle"]
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

def loop(secs, phase, rc, thr_override=None, print_every=1.0, freeze=False):
    """freeze=True: hold the plant motionless (clean static IC while the FC
    settles/calibrates/arms disarmed) -- sensors still stream."""
    last = 0.0
    t0 = time.time()
    f0 = _frames[0]
    # lockstep: phase durations count SIM time (frames), the run may be
    # faster or slower than the wall clock
    while (_frames[0] - f0 < secs / DT) if LOCKSTEP else (time.time() - t0 < secs):
        it0 = time.perf_counter()
        # NOTE: no GPS injection for now. Injecting our GPS (even only while
        # upright) biases the AHRS pitch via the COG/vel fusion -- revisit
        # together with the lock-quality-gated altitude-source feature.
        r = sim_step(m, plant.acc_mg(), plant.gyro_dps16(), rc, baro_pa=plant.baro_pa())
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
            if _man == "tvc_hang":
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
                  f"{rc[0]},{rc[1]},{rc[2]},{rc[3]},{rc[4]},{rc[5]},{rc[6]},{rc[7]},"
                  f"{_alt_cache[0]:.1f},{tvcp:.2f},{tvcy:.2f},{plant.xy()[0]:.1f},{plant.xy()[1]:.1f},"
                  f"{_gps_cache[0]},{_gps_cache[1]}\n")
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

print("=== SETTLE (AHRS an JSBSim angleichen, Plant eingefroren) ===")
loop(6, "settle", rc_ch(), freeze=True)
fr, fp, fy = fc_att(m); jr, jp, jy = plant.rpy()
print(f"Konventions-Check: FC {fr:+.1f}/{fp:+.1f} vs JS {jr:+.1f}/{jp:+.1f}  "
      f"({'OK' if abs(fr-jr)<15 and abs(fp-jp)<15 else 'MISMATCH -> Vorzeichen pruefen'})")

print("=== CAL (HITL-Stream laufen lassen bis bit9 weg) ===")
t0 = time.time()
while (arming_flags(m) & FLAG_CAL) and time.time() - t0 < 25:
    loop(1.0, "cal", rc_ch(angle=RC_HIGH), print_every=4, freeze=True)
print("cal fertig, flags=0x%X" % arming_flags(m))

print("=== ARM (Toggle-Zyklen bis ARMED) ===")
t0 = time.time()
while not (arming_flags(m) & FLAG_ARMED) and time.time() - t0 < 20:
    loop(1.0, "armL", rc_ch(thr=RC_LOW, arm=RC_LOW, angle=RC_HIGH), print_every=9, freeze=True)
    loop(1.2, "armH", rc_ch(thr=RC_LOW, arm=RC_HIGH, angle=RC_HIGH), print_every=9, freeze=True)
print("ARMED:", bool(arming_flags(m) & FLAG_ARMED), f"flags=0x{arming_flags(m):X}")

# released from the frozen IC only now: level + settle for a few seconds so
# the whole loop (plant, AHRS, controller) is in steady state before testing
print("=== ANGLE LEVEL (einschwingen aus sauberer Initialbedingung) ===")
loop(6, "level", rc_ch(thr=1650, arm=RC_HIGH, angle=RC_HIGH))   # 1450 = level trim (~50 kts, 0 m/min)

MAN = next((a for a in sys.argv[1:] if not a.startswith("--")), "inverted")
MAN_RC = {   # SEL detents: 1270 INVERT / 1510 KN L / 1750 KN R / 1985 HANG
    "inverted":    dict(sel=1270),
    "inverted_stick": dict(sel=1270),   # stick-offset carving around inverted
    "knife_left":  dict(sel=1510),
    "knife_right": dict(sel=1750),
    "hang":        dict(sel=1985),
    "roll_hold":   dict(invert=1575),                 # F ROLL band, own switch mid
    "loop_fig":    dict(angle=1300),                  # F LOOP band on the ANGLE channel
    "floor_dive":  dict(angle=RC_HIGH, invert=1900),  # FLOOR switch high
    "flat_spin":   dict(),                            # pro-spin sticks in ACRO, then ANGLE recovery
    "f_spin":      dict(angle=1575),                  # F SEQ: controlled flat spin figure
    "fspin_mode":  dict(invert=1300),                 # FLAT SPIN flight mode (pilot rudder)
    "tvc_hang":    dict(sel=1985),                    # prop hang on the TVC pusher delta
}[MAN]
thrM = 1500 if MAN in ("hang", "tvc_hang") else 1650   # level trim; holds start stable (hang: hover PID owns)

if MAN == "f_spin":
    # controlled flat spin figure: settle level -> stall kick (IMPULSE) ->
    # SPIN(2 turns, full rudder) with roll/pitch actively held flat ->
    # level hold with assist
    FIGSEG_END, FIGSEG_WAIT_TIME, FIGSEG_IMPULSE, FIGSEG_SPIN = 0, 5, 6, 8
    m.set_figure_segment(0, FIGSEG_WAIT_TIME, p3=1500, flags=1)
    m.set_figure_segment(1, FIGSEG_IMPULSE, p1=100, p2=100, p3=700)
    m.set_figure_segment(2, FIGSEG_SPIN, p1=2, p2=100, p3=12000)
    m.set_figure_segment(3, FIGSEG_WAIT_TIME, p3=4000, flags=1)
    m.set_figure_segment(4, FIGSEG_END)

# --- MANUAL: pilot flies by hand in ANGLE so the sticks visibly move,
#     then we flip the figure switch -> the sequence takes over ---
print("=== MANUAL (Pilot fliegt von Hand in ANGLE, Sticks bewegen sich) ===")
loop(3, "manual", rc_ch(thr=1650, arm=RC_HIGH, angle=RC_HIGH, ail=1250))
loop(3, "manual", rc_ch(thr=1650, arm=RC_HIGH, angle=RC_HIGH, ail=1750))
loop(3, "manual", rc_ch(thr=1650, arm=RC_HIGH, angle=RC_HIGH))

print(f"=== SEQUENCE {MAN} (Umschalten manuell -> Regler) ===")
if MAN == "flat_spin":
    # spin entry in ACRO: idle power, full up-elevator, full rudder -- the
    # stalled wing autorotates; then flip ANGLE back on: the controller
    # must catch the spin and level out
    loop(7, "spin-entry", rc_ch(thr=1000, arm=RC_HIGH, ele=2000, rud=2000), print_every=0.7)
    loop(12, "recover", rc_ch(thr=1650, arm=RC_HIGH, angle=RC_HIGH), print_every=0.7)
elif MAN == "floor_dive":
    # floor arms only after climbing above floor+margin (30+10 m over the
    # baro zero); then push over and HOLD the stick -- the floor must catch
    # and level AGAINST the held down-elevator, not because we let go
    loop(3, "arm-floor", rc_ch(thr=1700, arm=RC_HIGH, **MAN_RC), print_every=1)
    loop(8, "climb", rc_ch(thr=1900, arm=RC_HIGH, ele=1800, **MAN_RC), print_every=1)
    loop(14, "dive-held", rc_ch(thr=1700, arm=RC_HIGH, ele=1150, **MAN_RC), print_every=0.7)
    # contrast pass: floor switch OFF, same held push -> it punches through
    # the floor plane (climb high enough first, then keep pushing well past it)
    loop(8, "climb2", rc_ch(thr=1900, arm=RC_HIGH, ele=1800, **MAN_RC), print_every=1)
    loop(13, "dive-nofloor", rc_ch(thr=1700, arm=RC_HIGH, ele=1150, angle=RC_HIGH), print_every=0.7)
elif MAN in ("hang", "tvc_hang"):
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
elif MAN == "fspin_mode":
    # FLAT SPIN as a flight mode: box on holds the attitude flat, the
    # pilot's rudder drives the autorotation (idle throttle, full rudder),
    # releasing the rudder stops the rotation with the attitude still
    # held, releasing the box recovers to ANGLE
    loop(3, "flat-hold", rc_ch(thr=1650, arm=RC_HIGH, **MAN_RC), print_every=0.7)
    loop(10, "spin-rud", rc_ch(thr=1000, arm=RC_HIGH, rud=2000, **MAN_RC), print_every=0.7)
    loop(5, "rud-release", rc_ch(thr=1650, arm=RC_HIGH, **MAN_RC), print_every=0.7)
    loop(5, "exit", rc_ch(thr=1650, arm=RC_HIGH, angle=RC_HIGH), print_every=0.7)
elif MAN == "f_spin":
    # figure runs on its own; idle throttle for the spin itself, then power
    # back for the level recovery segment
    loop(3, "f-settle", rc_ch(thr=1650, arm=RC_HIGH, **MAN_RC), print_every=0.7)
    loop(14, "f-spin", rc_ch(thr=1000, arm=RC_HIGH, **MAN_RC), print_every=0.7)
    loop(6, "f-level", rc_ch(thr=1650, arm=RC_HIGH, **MAN_RC), print_every=0.7)
    loop(5, "exit", rc_ch(thr=1650, arm=RC_HIGH, angle=RC_HIGH), print_every=0.7)
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
    # error returning to zero afterwards
    loop(8, MAN, rc_ch(thr=thrM, arm=RC_HIGH, angle=RC_LOW, **MAN_RC), print_every=0.7)
    plant.set_wind(down_ms=3.0)
    loop(4, "gust", rc_ch(thr=thrM, arm=RC_HIGH, angle=RC_LOW, **MAN_RC), print_every=0.7)
    plant.set_wind()
    loop(12, MAN, rc_ch(thr=thrM, arm=RC_HIGH, angle=RC_LOW, **MAN_RC), print_every=0.7)

fr, fp, fy = fc_att(m); jr, jp, jy = plant.rpy()
print(f"FINAL: FC roll {fr:+.1f}  JS roll {jr:+.1f}  (Erfolg wenn |roll| ~ 180)")
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
