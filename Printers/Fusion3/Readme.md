# Reset Heater Fault

During Pause:

- M562 P1 (tool1heater)
- G1 Z_ or G1 R1

__Z layer it left off on!__

- M84 E -> disables extruder drive
- G91 -> relative module
