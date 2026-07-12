# Video → Figure-Script Prompt (Gemini / AI mode)

Paste this prompt together with a flight-video URL (Gemini accepts YouTube
links directly) to extract a figure script for `figure_script.py`. The
output is a STARTING POINT — validate in the SITL bench, then trim on the
real airframe.

---

Analyze the RC aerobatics in this video. Break the flight into a sequence
of maneuvers with timestamps, then express the sequence in the following
JSON format. Only use these segment types:

- `{"type": "roll", "degrees": <signed>, "assist": true|false}` — roll
  rotation, cumulative (half roll = 180, full = 360, opposite direction
  negative). Set "assist": true when the pilot visibly holds altitude
  through the roll.
- `{"type": "pitch", "degrees": <signed>}` — pitch rotation, cumulative
  (full loop = 360, half loop / Immelmann first half = 180, 45-degree
  up-line entry = 45).
- `{"type": "hold", "roll_deg": <abs>, "pitch_deg": <abs>, "ms": <n>,
  "assist": true|false}` — hold an absolute attitude (inverted pass =
  roll 180, knife edge = roll +-90, 45-line = pitch 45).
- `{"type": "wait_time", "ms": <n>, "assist": true}` — level pause
  between figures.
- `{"type": "wait_alt", "altitude_m": <n>, "tolerance_m": 3}` — climb
  gate before a figure that needs entry altitude; altitude_m minimum 20
  (use 30 when the video flies lower — the bench flies with margin).

Rules:
- Maximum 15 segments; pick the clearest continuous sequence if the video
  has more.
- Estimate rotation rates only qualitatively; rates are configured
  separately (fig_roll_rate / fig_loop_rate), so do NOT emit rates.
- Post-stall entries (snap roll, spin entry) map to
  `{"type": "impulse", "pitch_pct": <-100..100>, "yaw_pct": <-100..100>,
  "ms": <50..2000>}` — a full-deflection open-loop kick; the NEXT segment
  catches the resulting attitude. Sustained chaotic maneuvers (blender,
  lomcovak) remain unsupported — list them with timestamps.
- `{"type": "wait_pos", "radius_m": <n>, "max_bank_deg": 30}` — fly back
  toward the home point until within the radius (airspace containment
  between figures).
- Output: first a timestamped maneuver table, then exactly one JSON code
  block in the format above.
