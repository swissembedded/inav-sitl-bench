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

"""MSP_SIMULATOR v3 payload pack/unpack (INAV HITL protocol, fc_msp.c).

Request layout after [version u8][flags u16] (all blocks always present in
the stream, options only decide whether the FC uses them):
  GPS      : fix u8, numSat u8, lat i32, lon i32, alt i32,
             gspd i16, gcourse i16, velNED i16*3            (24 B)
  attitude : roll,pitch,yaw i16 decideg (ignored with HITL_USE_IMU)  (6 B)
  acc      : i16*3, milli-g                                  (6 B)
  gyro     : i16*3, dps*16                                   (6 B)
  baro     : u32 Pa                                          (4 B)
  mag      : i16*3, uT*20                                    (6 B)
  v3 tail  : rangefinder u16, current u16,
             rcInput u16*8, rssi u16                         (22 B)
(no vbat/airspeed bytes: we do not set HITL_EXT_BATTERY_VOLTAGE /
 HITL_AIRSPEED / HITL_EXTENDED_FLAGS)

Reply:
  stabilized roll,pitch,yaw,throttle i16 (+/-500, throttle -500 disarmed),
  debugIndex u8 (bit7 airplane, bit6 armed), debug u32,
  attitude roll,pitch,yaw i16 decideg, OSD blob (ignored)
"""
from __future__ import annotations

import struct
from dataclasses import dataclass

from msp import MspClient, MSP_SIMULATOR

HITL_ENABLE = 1 << 0
HITL_MUTE_BEEPER = 1 << 2
HITL_USE_IMU = 1 << 3
HITL_HAS_NEW_GPS_DATA = 1 << 4
HITL_SIM_RC_INPUT = 1 << 11

BENCH_FLAGS = HITL_ENABLE | HITL_MUTE_BEEPER | HITL_USE_IMU | HITL_SIM_RC_INPUT


@dataclass
class SimStepResult:
    stab_roll: float      # -1..+1
    stab_pitch: float
    stab_yaw: float
    stab_throttle: float  # -1..+1 (mixer input range)
    airplane: bool
    armed: bool
    att_roll_deg: float   # FC's own attitude estimate
    att_pitch_deg: float
    att_yaw_deg: float


def pack_request(acc_mg: tuple[int, int, int],
                 gyro_dps16: tuple[int, int, int],
                 rc_channels_us: list[int],
                 baro_pa: int = 101325,
                 mag: tuple[int, int, int] = (0, 0, 0),
                 flags: int = BENCH_FLAGS,
                 gps: dict | None = None) -> bytes:
    assert len(rc_channels_us) == 8
    # a diverged plant (crash, numerical blow-up) produces pressures far
    # outside the packable range; fail with a diagnosis instead of an
    # opaque struct.error deep in the pack call
    if not (0 <= baro_pa < 2**32):
        raise RuntimeError(
            f"plant diverged: baro_pa={baro_pa} is outside any physical range "
            "(the airframe left the envelope - check the maneuver/log, this "
            "is not a packing bug)")
    if gps is not None:
        flags |= HITL_HAS_NEW_GPS_DATA
    p = struct.pack("<BH", 3, flags)
    if gps is not None:
        p += struct.pack("<BBiiihhhhh",
                         gps.get("fixType", 2),          # GPS_FIX_3D == 2
                         gps.get("numSat", 12),
                         gps["lat_e7"], gps["lon_e7"], gps["alt_cm"],
                         gps["speed_cms"], gps["course_dd"], *gps["vel_ned_cms"])
    else:
        p += struct.pack("<BBiiihhhhh", 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)   # GPS block, unused
    p += struct.pack("<hhh", 0, 0, 0)                               # euler att, unused (HITL_USE_IMU)
    p += struct.pack("<hhh", *[int(v) for v in acc_mg])
    p += struct.pack("<hhh", *[int(v) for v in gyro_dps16])
    p += struct.pack("<I", baro_pa)
    p += struct.pack("<hhh", *mag)
    p += struct.pack("<HH", 0xFFFF, 0)                               # rangefinder off, current
    p += struct.pack("<8H", *[int(v) for v in rc_channels_us])
    p += struct.pack("<H", 1000)                                     # rssi
    return p


def unpack_reply(payload: bytes) -> SimStepResult:
    sr, sp, sy, st = struct.unpack_from("<hhhh", payload, 0)
    dbg_index = payload[8]
    dbg_value = struct.unpack_from("<i", payload, 9)[0]
    (ar, ap, ay) = struct.unpack_from("<hhh", payload, 13)
    r = SimStepResult(
        stab_roll=sr / 500.0, stab_pitch=sp / 500.0, stab_yaw=sy / 500.0,
        stab_throttle=st / 500.0,
        airplane=bool(dbg_index & 128), armed=bool(dbg_index & 64),
        att_roll_deg=ar / 10.0, att_pitch_deg=ap / 10.0, att_yaw_deg=ay / 10.0,
    )
    r.debug = (dbg_index & 7, dbg_value)   # (debug slot, value), cycles 0..7
    return r


def sim_step(msp: MspClient, acc_mg, gyro_dps16, rc_us, **kw) -> SimStepResult:
    reply = msp.request(MSP_SIMULATOR, pack_request(acc_mg, gyro_dps16, rc_us, **kw))
    return unpack_reply(reply)
