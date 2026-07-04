This first update adds the four core Python programs that make CarloBot run.

- Pro_CAM.py — runs on the OpenMV H7 camera with LED shield. Detects red/green 
  traffic signs using LAB color thresholds and reports color + position 
  (left/right) over UART to the ESP32.

- Con_ESP32.py — runs on the ESP32. Reads the 3 TF-Luna LiDAR sensors 
  (front, left, right) over SoftI2C, receives the color/position data from 
  the camera over UART, and relays everything to the LEGO Inventor Hub via 
  PUPRemote (Port D).

- Ope_Chall.py — runs on the LEGO Inventor Hub. Controls the robot during 
  the Open Challenge: detects driving direction, follows the wall using 
  LiDAR distance, and completes 3 laps (12 corners) before stopping.

- Obs_Chall.py — runs on the LEGO Inventor Hub. Extends the Open Challenge 
  logic to react to the traffic signs detected by the camera, adjusting 
  the lane position (right for red, left for green).

Hardware connected in this update: 3x TF-Luna LiDAR (SoftI2C) + OpenMV H7 
camera (UART) → ESP32 → LEGO Inventor Hub (PUPRemote).

No mechanical or documentation changes in this commit — code only.