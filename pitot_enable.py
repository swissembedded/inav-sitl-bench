"""Enable the SITL pitot (FAKE driver = HITL stream injects truth airspeed)
and persist, so the sensor initialises on the next boot. pitot_lever_arm is
set live by jsbsim_fly.py --set (read every getAirspeedEstimate call)."""
import struct
from msp import MspClient
m = MspClient()
m.set_setting("pitot_hardware", struct.pack("<B", 5))
m.save_eeprom()
print("pitot enabled (hardware=5), eeprom saved, SITL reboots")
