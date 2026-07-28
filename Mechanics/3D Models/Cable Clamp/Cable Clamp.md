# Cable Clamp

Mechanical part used to secure the wiring on CarloBot, particularly the wires running to the ESP32. This folder documents three design iterations.

## Files

- **Cable Carlobot 1.stl** — first prototype. Dimensions: 44.00 x 32.38 x 15.00 mm. This version was too large: cables slipped out easily during testing.

- **Cable Carlobot 2.stl** — second prototype. Dimensions: 44.00 x 32.38 x 8.00 mm. Height was reduced to 8 mm (matching a standard LEGO piece height), which also reduced print time. Length and width were unchanged from the first prototype.

- **Cable Carlobot 3.stl** — third prototype (current version). Dimensions: 
  30.00 x 28.00 x 8.00 mm. Length and width were reduced from the second prototype while keeping the same height. This solved the cable-slipping issue specifically for the wires going to the ESP32.

## Design rationale

Each iteration targeted a specific problem found during testing: first reducing height for faster printing, then reducing the footprint to grip the wires more tightly. All three files are kept in this folder to document the design process, not just the final part.