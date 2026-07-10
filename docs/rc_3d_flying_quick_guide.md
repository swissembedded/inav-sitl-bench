# RC 3D Flying Quick-Guide (Mode 2)

**Stick Assignment**:
*   **Left Stick**: Throttle / Rudder
*   **Right Stick**: Aileron / Elevator

*Note: All inputs are described from the pilot's perspective inside the cockpit.*

---

## 1. Hover & Torquing
*Vehicle hangs vertically in the air, sustained entirely by the propeller blast.*

*   **Throttle**: Modulate precisely. Keep it at the exact sweet spot to hold altitude without climbing or sinking.
*   **Elevator**: Counteract any pitching. If the nose falls forward, pull **Down**. If it falls backward, push **Up**.
*   **Rudder**: Counteract any yawing. If the nose drifts left, steer **Right**. For a **Torque Roll**, let the engine torque rotate the plane counter-clockwise (or push **Left** to accelerate it).
*   **Aileron**: Use minimal input. Keep it centered to allow torquing, or steer **Right** to stop the rotation for a steady **Hover**.

---

## 2. Harrier
*Aircraft flies forward at a steep, stalled angle of attack (nose up at roughly 45 deg).*

*   **Elevator**: Hold the stick **Down** (pulled back) constantly to maintain the high alpha attitude.
*   **Throttle**: Controls your **altitude**. Add power to climb, reduce power to descend.
*   **Rudder**: Your primary tool for **steering** and turning. Ailerons are highly inefficient in a Harrier and cause wing rock.
*   **Aileron**: Use only for minor, quick corrections to keep the wings level.

---

## 3. Knife Edge
*The plane flies straight and level while locked at a 90-degree bank angle.*

*   **Aileron**: Deflect briefly to roll the plane **90 degrees** onto its side, then return to neutral.
*   **Rudder**: This is your **primary lift control**. Deflect it hard to the opposite side of the low wing (e.g., if rolled left, push the stick **Right**) to keep the nose up.
*   **Elevator**: Pitch coupling is common here. Use minor corrections to prevent the plane from pulling toward the canopy or pushing toward the belly.
*   **Throttle**: Increase slightly above cruising speed to compensate for the lower aerodynamic efficiency of the fuselage.

---

## Mapping to the bench maneuvers

What the orientation-hold controller must reproduce, per figure:

| Guide figure | Bench maneuver | What the controller owns |
| --- | --- | --- |
| Hover & Torquing | `hang` | attitude (nose vertical) + hover throttle PID (altitude) |
| Knife Edge | `knife_left` / `knife_right` | 90 deg bank + top rudder for fuselage lift + pitch-coupling trim |
| Inverted (not in guide: sustained inverted level) | `inverted` | roll 180 deg + inverted pitch trim, altitude assist |

The replay videos plot exactly this ownership: pilot sticks stay centered
(controller IN) while the control-surface commands (controller OUT) do the
work described above for the human pilot.
