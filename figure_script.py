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

"""Program an INAV figure sequence from a JSON script.

The missing piece of the video-to-sequence pipeline: a human (or an LLM
analyzing a flight video, see PROMPT.md) writes a JSON figure script, this
tool validates it and programs it into the FC via MSP2_INAV_SET_FIGURE_SEQUENCE.
Fly it with the FIGURE SEQ box.

Usage:
  python figure_script.py immelmann.json            # program via MSP (tcp:5760)
  python figure_script.py immelmann.json --dry-run  # validate + print only

Script format (segments run in order, max 16 incl. terminator):
{
  "name": "gated immelmann",
  "segments": [
    {"type": "wait_alt",  "altitude_m": 40, "tolerance_m": 3},
    {"type": "pitch",     "degrees": 180},
    {"type": "roll",      "degrees": 180},
    {"type": "wait_time", "ms": 2000, "assist": true},
    {"type": "hold",      "roll_deg": 0, "pitch_deg": 45, "ms": 1500, "assist": false}
  ]
}
Rotations are cumulative on the running attitude baseline (Immelmann =
pitch +180 then roll +180). "assist" enables the altitude assist for the
segment (rotations: roll only; pitch rotations never use it).
"""
from __future__ import annotations

import json
import sys

from msp import MspClient

(FIGSEG_END, FIGSEG_ROLL, FIGSEG_PITCH, FIGSEG_HOLD, FIGSEG_WAIT_ALT,
 FIGSEG_WAIT_TIME, FIGSEG_IMPULSE, FIGSEG_WAIT_POS) = range(8)
FLAG_ASSIST = 1
MAX_SEGMENTS = 16


def compile_script(script: dict) -> list[tuple]:
    """Return list of (type, p1, p2, p3, flags) tuples, validated."""
    segs = script.get("segments")
    if not isinstance(segs, list) or not segs:
        raise ValueError("script needs a non-empty 'segments' list")
    if len(segs) > MAX_SEGMENTS - 1:
        raise ValueError(f"too many segments ({len(segs)}), max {MAX_SEGMENTS - 1} plus terminator")

    out = []
    for i, s in enumerate(segs):
        t = s.get("type")
        assist = FLAG_ASSIST if s.get("assist") else 0
        if t == "roll":
            d = int(s["degrees"])
            if not -720 <= d <= 720 or d == 0:
                raise ValueError(f"segment {i}: roll degrees {d} out of range")
            out.append((FIGSEG_ROLL, d, 0, 0, assist))
        elif t == "pitch":
            d = int(s["degrees"])
            if not -720 <= d <= 720 or d == 0:
                raise ValueError(f"segment {i}: pitch degrees {d} out of range")
            out.append((FIGSEG_PITCH, d, 0, 0, 0))
        elif t == "hold":
            ms = int(s["ms"])
            if not 100 <= ms <= 30000:
                raise ValueError(f"segment {i}: hold ms {ms} out of range")
            out.append((FIGSEG_HOLD, int(s.get("roll_deg", 0)), int(s.get("pitch_deg", 0)), ms, assist))
        elif t == "wait_alt":
            alt = int(s["altitude_m"])
            if not 5 <= alt <= 500:
                raise ValueError(f"segment {i}: wait_alt altitude {alt} out of range")
            out.append((FIGSEG_WAIT_ALT, alt, int(s.get("tolerance_m", 3)), 0, 0))
        elif t == "wait_time":
            ms = int(s["ms"])
            if not 100 <= ms <= 30000:
                raise ValueError(f"segment {i}: wait_time ms {ms} out of range")
            out.append((FIGSEG_WAIT_TIME, 0, 0, ms, assist))
        elif t == "impulse":
            ms = int(s["ms"])
            if not 50 <= ms <= 2000:
                raise ValueError(f"segment {i}: impulse ms {ms} out of range")
            out.append((FIGSEG_IMPULSE, int(s.get("pitch_pct", 0)), int(s.get("yaw_pct", 0)), ms, 0))
        elif t == "wait_pos":
            r = int(s["radius_m"])
            if not 10 <= r <= 2000:
                raise ValueError(f"segment {i}: wait_pos radius {r} out of range")
            out.append((FIGSEG_WAIT_POS, r, int(s.get("max_bank_deg", 30)), 0, 0))
        else:
            raise ValueError(f"segment {i}: unknown type '{t}'")
    out.append((FIGSEG_END, 0, 0, 0, 0))
    return out


NAMES = {0: "END", 1: "ROLL", 2: "PITCH", 3: "HOLD", 4: "WAIT_ALT", 5: "WAIT_TIME",
         6: "IMPULSE", 7: "WAIT_POS"}


def main():
    path = sys.argv[1]
    dry = "--dry-run" in sys.argv
    with open(path, encoding="utf-8") as f:
        script = json.load(f)
    compiled = compile_script(script)

    print(f"sequence '{script.get('name', path)}': {len(compiled) - 1} segments")
    for i, (t, p1, p2, p3, fl) in enumerate(compiled):
        extra = " +assist" if fl & FLAG_ASSIST else ""
        print(f"  [{i:2d}] {NAMES[t]:9s} p1={p1:5d} p2={p2:5d} p3={p3:5d}{extra}")
    if dry:
        return

    msp = MspClient()
    for i, seg in enumerate(compiled):
        msp.set_figure_segment(i, *seg)
    for i in range(len(compiled), MAX_SEGMENTS):
        msp.set_figure_segment(i, FIGSEG_END)
    msp.save_eeprom()
    msp.close()
    print("programmed + saved. Fly it with the FIGURE SEQ box.")


if __name__ == "__main__":
    main()
