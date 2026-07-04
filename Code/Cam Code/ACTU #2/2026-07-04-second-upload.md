This update replaces the ESP32 firmware with a non-blocking version and implements the actual traffic-sign navigation logic for the Obstacle Challenge.

Changed files:

* Con\_ESP32.py — full rewrite. The previous version blocked the main loop
with sleep\_ms() between each TF-Luna command and its response, and parsed
camera data with string decode()/split(). The new version:
· Reads the 3 TF-Luna sensors with a non-blocking state machine
(TF\_ESTADO\_LIBRE / TF\_ESTADO\_ESPERANDO), so no sensor read stalls the loop.
· Validates each TF-Luna packet with its checksum before accepting the
distance value.
· Parses the camera's UART line directly on bytes (no decode()/split()),
combining color + position into a single value (0, 11, 12, 21, 22) sent
to the Hub over one PUPRemote channel ("cam").
· Applies timeouts: camera value resets to 0 if OpenMV goes silent for
250 ms, and each LiDAR resets to 0 if its sensor stops responding for
300 ms.
· Uses PUPRemoteSensor to expose "cam" and "dist" channels to the Hub,
replacing the previous debug-only print statements.
* Obs\_Chall.py — expanded from a wall-following test stub into the actual
Obstacle Challenge behavior. Added:
· verde() / rojo() — steering routines that react to a detected green or
red pillar (keep left / keep right).
· navegacion() — reads the camera channel and decides which routine to run
based on the color and position received.
· curva\_r() / curva\_v() / curvas() — corner-handling routines chained after
each straight section.

Unchanged in this update: Ope\_Chall.py and Pro\_CAM.py (no functional
differences from the previous commit).

