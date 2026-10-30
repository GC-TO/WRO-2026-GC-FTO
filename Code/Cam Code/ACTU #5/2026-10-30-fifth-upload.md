# Obstacle Challenge: parking maneuvers, tunable speed constants, and corner-correction overhaul

This update replaces Obs_Chall.py with a major revision aimed at making the 
full run (including parking) reliable. Con_ESP32.py, Ope_Chall.py, and 
Pro_CAM.py are unchanged from the previous upload.

Added:

- Five parking-lot maneuvers: parqueo2(), parqueo3(), parqueo4(), 
  parqueo5(), parqueo6(). Each backs the car into the parking space using a 
  mix of heading-lock driving and front/left distance checks, then stops 
  the motors for the rest of the run. navegacion() and navegacion_ccw() now 
  call the matching parqueo function once "daniel" (the running corner-count 
  angle) passes a threshold (±1000), signaling the robot has entered the 
  final parking section.

- jaja = StopWatch(): a timer started at the top of the file and printed 
  (jaja.time()) at the end of main() and inside each parqueo function, to 
  measure total run time during testing.

- A block of named speed constants (SPEED_200 through SPEED_850) that 
  replace the hardcoded speed numbers used throughout the file. Values that 
  were only slightly apart from a shared value (e.g. 450 vs 400, 550 vs 
  500) were folded into one constant so they tune together.

- correctionClock (90.40) and correctionCounter (90.63): replace the fixed 
  90 / 90.5 degree correction added to "daniel" after each corner. These 
  come from measured drift and are now also used for the parking-section 
  threshold checks (e.g. checking daniel against correctionCounter*4, *8, 
  *12 instead of hardcoded values like -362, -724, -1086), which is more 
  robust against small floating-point drift than exact equality checks 
  were.

Changed:

- motor_traccion moved from Port.E to Port.C.
- Main loop now runs 12 sections instead of 11 (vueltas < 12), to cover the 
  added parking segment.
- salida_clock() is now structurally symmetric with salida_counter(): both 
  use move_for_distance() and look_for_block() for the initial exit from 
  the start box instead of only one of the two directions having that 
  logic.
- look_for_block() now takes the drive speed as a parameter instead of a 
  hardcoded 300, and the creep-forward loop was restructured to check the 
  camera more consistently while creeping.
- drive_with_heading_lock() now flips the sign of the steering correction 
  when driving in reverse (drive_speed < 0), fixing steering direction 
  during the backward segments used by the new parking maneuvers.
- giro_con_imu(): MIN_SPEED raised from 200 to 250.
- Several corner-handling routines (rojo_ccw, verde_ccw, rojo2_ccw, 
  verde2_ccw, and their clockwise counterparts) retuned: turn angles, 
  approach distances, and in some cases switched from sigue_pared() to a 
  single drive_with_heading_lock() call for a more predictable approach.

Known issues carried over from the previous commit:
- salida_bloque() is still dead code — references "color" without reading 
  it first, and nothing in the file calls it.