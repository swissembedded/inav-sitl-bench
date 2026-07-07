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

"""Minimal MSPv2 client over TCP for INAV SITL.

Only what the orientation-hold bench needs: MSPv2 framing (native $X),
CLI passthrough for one-time provisioning, and helpers for the handful of
commands the bench uses.
"""
from __future__ import annotations

import socket
import struct
import time

MSP2_COMMON_SETTING_INFO = 0x1003
MSP2_COMMON_SET_SETTING = 0x1004
MSP_SIMULATOR = 0x201F
MSP_API_VERSION = 1
MSP_STATUS = 101
MSP_ATTITUDE = 108
MSP_BOXIDS = 119
MSP_SET_MODE_RANGE = 35
MSP_MODE_RANGES = 34
MSP_EEPROM_WRITE = 250
MSP_REBOOT = 68
MSP_FEATURE = 36
MSP_SET_FEATURE = 37
MSP2_INAV_SET_SERVO_MIXER = 0x2021
MSP2_INAV_SET_FIGURE_SEQUENCE = 0x2241
MSP_SERVO = 103


def crc8_dvb_s2(data: bytes, crc: int = 0) -> int:
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


class _SerialAsSocket:
    """Minimal socket-like wrapper so MspClient works on a real FC via COM
    port (e.g. MspClient('COM7') for the level-1 hardware test)."""

    def __init__(self, device: str, baud: int, timeout: float):
        import serial
        self.ser = serial.Serial(device, baud, timeout=timeout)

    def sendall(self, data):
        self.ser.write(data)

    def recv(self, n):
        data = self.ser.read(max(1, min(n, self.ser.in_waiting or 1)))
        if not data:
            raise TimeoutError("serial read timeout")
        return data

    def settimeout(self, t):
        self.ser.timeout = t

    def close(self):
        self.ser.close()


class MspClient:
    def __init__(self, host: str = "127.0.0.1", port: int = 5760, timeout: float = 3.0,
                 baud: int = 115200):
        if host.upper().startswith("COM") or host.startswith("/dev/"):
            self.sock = _SerialAsSocket(host, baud, timeout)
        else:
            self.sock = socket.create_connection((host, port), timeout=timeout)
            self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._rx = b""

    def close(self):
        self.sock.close()

    # ---- MSPv2 framing ----
    def send(self, cmd: int, payload: bytes = b""):
        body = struct.pack("<BHH", 0, cmd, len(payload)) + payload
        frame = b"$X<" + body + bytes([crc8_dvb_s2(body)])
        self.sock.sendall(frame)

    def _read_exact(self, n: int) -> bytes:
        while len(self._rx) < n:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise ConnectionError("SITL closed connection")
            self._rx += chunk
        out, self._rx = self._rx[:n], self._rx[n:]
        return out

    def recv(self) -> tuple[int, bytes, bool]:
        """Return (cmd, payload, is_error). Skips non-MSP noise bytes."""
        while True:
            b = self._read_exact(1)
            if b != b"$":
                continue
            if self._read_exact(1) != b"X":
                continue
            direction = self._read_exact(1)
            hdr = self._read_exact(5)
            _flag, cmd, size = struct.unpack("<BHH", hdr)
            payload = self._read_exact(size)
            crc = self._read_exact(1)[0]
            if crc8_dvb_s2(hdr + payload) != crc:
                raise IOError(f"MSP CRC mismatch on cmd {cmd}")
            return cmd, payload, direction == b"!"

    def request(self, cmd: int, payload: bytes = b"", retries: int = 3) -> bytes:
        for _ in range(retries):
            self.send(cmd, payload)
            rcmd, rpayload, err = self.recv()
            if rcmd == cmd:
                if err:
                    raise IOError(f"MSP error reply for cmd 0x{cmd:X}")
                return rpayload
        raise IOError(f"no matching reply for cmd 0x{cmd:X}")

    # ---- helpers ----
    def api_version(self) -> tuple[int, int, int]:
        p = self.request(MSP_API_VERSION)
        return p[0], p[1], p[2]

    def attitude_deg(self) -> tuple[float, float, float]:
        p = self.request(MSP_ATTITUDE)
        roll, pitch, yaw = struct.unpack("<hhh", p[:6])
        return roll / 10.0, pitch / 10.0, yaw * 1.0

    def set_setting(self, name: str, raw_value: bytes) -> None:
        """MSP2_COMMON_SET_SETTING: zero-terminated name + raw value bytes."""
        self.request(MSP2_COMMON_SET_SETTING, name.encode() + b"\x00" + raw_value)

    def setting_info(self, name: str) -> bytes:
        return self.request(MSP2_COMMON_SETTING_INFO, name.encode() + b"\x00")

    def set_mode_range(self, index: int, box_permanent_id: int, aux_channel: int,
                       start_pwm: int, end_pwm: int) -> None:
        """aux_channel is 0-based AUX index (AUX1 = channel 5 overall = 0 here)."""
        payload = struct.pack("<BBBBB", index, box_permanent_id, aux_channel,
                              (start_pwm - 900) // 25, (end_pwm - 900) // 25)
        self.request(MSP_SET_MODE_RANGE, payload)

    def set_servo_mixer_rule(self, index: int, servo: int, input_source: int,
                             rate: int = 100, speed: int = 0) -> None:
        payload = struct.pack("<BBBhBB", index, servo, input_source, rate, speed, 0xFF)
        self.request(MSP2_INAV_SET_SERVO_MIXER, payload)

    def set_figure_segment(self, index: int, seg_type: int, p1: int = 0, p2: int = 0,
                           p3: int = 0, flags: int = 0) -> None:
        payload = struct.pack("<BBhhhB", index, seg_type, p1, p2, p3, flags)
        self.request(MSP2_INAV_SET_FIGURE_SEQUENCE, payload)

    def servos_us(self) -> list[int]:
        p = self.request(MSP_SERVO)
        return [struct.unpack_from("<H", p, i)[0] for i in range(0, len(p), 2)]

    def enable_feature(self, bit: int) -> None:
        mask = struct.unpack("<I", self.request(MSP_FEATURE))[0]
        self.request(MSP_SET_FEATURE, struct.pack("<I", mask | bit))

    def save_eeprom(self) -> None:
        self.request(MSP_EEPROM_WRITE)

    # ---- CLI passthrough (provisioning only) ----
    def cli(self, commands: list[str], settle: float = 0.4) -> str:
        """Enter CLI, run commands, exit/save. Consumes the MSP session!
        Only use on a dedicated connection; 'save'/'exit' reboots SITL."""
        self.sock.sendall(b"#")
        time.sleep(settle)
        out = b""
        for c in commands:
            self.sock.sendall(c.encode() + b"\n")
            time.sleep(settle)
            try:
                self.sock.settimeout(0.5)
                while True:
                    chunk = self.sock.recv(4096)
                    if not chunk:
                        break
                    out += chunk
            except (TimeoutError, socket.timeout):
                pass
            finally:
                self.sock.settimeout(3.0)
        return out.decode(errors="replace")
