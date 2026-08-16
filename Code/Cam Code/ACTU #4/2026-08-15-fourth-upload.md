# Obstacle Challenge: bidirectional navigation + camera calibration update

This update replaces Obs_Chall.py with a major expansion that handles both 
lap directions instead of only clockwise, and updates the camera's 
detection settings.

Changed files:

- Obs_Chall.py — added:
  · main() — reads left/right distance once at the start to decide the lap 
    direction, then dispatches to the correct set of routines (clockwise 
    or counter-clockwise) for all 11 sections of the run.
  · A full counter-clockwise mirror of every navigation function: 
    verde_ccw(), rojo_ccw(), rojo2_ccw(), verde2_ccw(), nada_ccw(), 
    nada2_ccw(), navegacion_ccw(), curva_r_ccw(), curva_v_ccw(), 
    curvas_ccw(). Turn angles and steering are flipped relative to their 
    clockwise counterparts.
  · salida_counter() / salida_clock() — starting-direction routines that 
    read the camera right after leaving the parking lot to decide which 
    branch to enter first.
  · move_for_distance(speed, distance, angle) — new helper that converts a 
    target distance in cm into drive time using wheel circumference.
  · Sensor mapping bug fixed inside drive_with_heading_lock(): sensor 2 
    (front) was previously a placeholder that always returned False; now 
    sensor 1 = front, 2 = left, 3 = right, all correctly implemented.
  · Fixed the "danile" typo from the previous commit.
  · Still unresolved: salida_bloque() is dead code — references "color" 
    without reading it first, and nothing in the file calls it. Needs 
    removal or a fix in a future commit.

- Pro_CAM.py — camera calibration update:
  · Sensor resolution raised from QQVGA (160x120) to QVGA (320x240).
  · ROI_LEFT and ROI_RIGHT resized and repositioned to match the new, 
    larger frame (from 70x8 px near the top of the frame to 140x16 px 
    lower in the frame).
  · RED_THRESHOLD widened significantly (L: 30-85 to 0-58, A/B: near full 
    range) to detect red pillars under a broader range of lighting 
    conditions.
  · DEBUG and DRAW_DEBUG both set to True for on-field tuning; should be 
    set back to False before the actual competition runs, since debug 
    printing and rectangle drawing cost processing time per frame.
  · Note: some in-file comments still refer to "QQVGA" near the resolution 
    and ROI settings — these are now outdated and should be updated to 
    avoid confusion, since the sensor actually runs at QVGA in this version.

Unchanged in this update: Con_ESP32.py, Ope_Chall.py — same versions as 
the previous upload.