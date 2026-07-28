# ESP32 Hub reconnection + Obstacle Challenge state machine + Open Challenge tuning

This update adds automatic Hub reconnection to the ESP32 firmware, rewrites 
the Obstacle Challenge into a full state machine covering parking-lot exit 
and corner logic, and tunes the Open Challenge corner behavior.

Changed files:

- Con_ESP32.py — replaced with a version that adds robustness to the 
  PUPRemote connection with the LEGO Hub. The previous version only called 
  pr.process() in the loop without checking if a connection actually 
  existed. The new version:
  · Adds max_packet_size=16 to PUPRemoteSensor, matching Pybricks' 
    recommended configuration for this channel setup.
  · Waits and confirms PUPRemote has negotiated a connection with the Hub 
    on startup (conectar_hub()), instead of assuming it's connected.
  · Detects a dropped connection every loop cycle and reconnects 
    automatically (Hub restart, low battery, interference), instead of 
    silently sending data to a channel nobody is reading.
  · Re-sends the last known camera and distance values right after a 
    reconnection (restaurar_canales()), so the Hub doesn't start with 
    stale/zeroed data.
  · Resets timing references after a reconnection so timeouts don't 
    misfire due to the time spent disconnected.

- Obs_Chall.py — replaced with a full state-machine rewrite. The previous 
  version only reacted to a pillar once the camera saw it; this version 
  adds the complete lap logic: parking-lot exit, corner handling, and 
  active searching when no pillar is visible yet.
  · main() — reads left/right distance at the start to decide initial 
    turning direction (salida_counter() for counter-clockwise start, 
    salida_clock() for clockwise start).
  · salida_counter() / salida_clock() — handle exiting the parking lot 
    section for each starting direction.
  · look_for_block() — creeps the car forward while polling the camera, 
    until either a pillar is detected or a distance limit is reached.
  · navegacion() — rewritten as a state machine using a new "probot" 
    variable (0-3) that tracks how many pillars have already been handled 
    in the current section.
  · curva_r() / curva_v() — corner routines now branch based on "probot", 
    deciding whether to keep searching for another pillar after the turn.
  · Main loop now runs a full "while vueltas < 11" cycle instead of two 
    fixed calls, covering all 3 laps plus the parking-lot sections.

- Ope_Chall.py — tuning and connection changes, no new logic branches.
  · PUPRemoteHub port changed from Port.D to Port.F.
  · Startup sequence now drives forward using the front sensor as a stop 
    condition (drive_with_heading_lock) before checking left/right 
    distance, instead of starting the direction check immediately.
  · Corner turn angle reduced from 60° to 50°, confirmation count reduced 
    from 5 to 3, post-turn wait reduced from 700 ms to 600 ms, and the 
    wall-following speed reduced from 700 to 500.
  · Removed a debug-only pre-turn front-distance check.

Unchanged in this update: Pro_CAM.py (no functional differences from the 
previous commit).