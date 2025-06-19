## [v0.1.2] – 2025-06-20
---
Added:
- **`detection_with servo mapping (mock).py`**
  - Bounding boxes are used to extract `(cx, cy)` center coordinates and marks it inside the bounding box.
  - Center coordinates are then mapped to servo angles using a custom `mapping_value()` function.
  - Includes a `servo_output()` function that prints target angles to simulate servo motion.
  - No actual hardware GPIO control has been added yet. This is a mock version to see the angle conversion.
---

### Servo Angle Mapping Logic

This version introduces a basic range mapping function to translate pixel positions (like `cx`, `cy`) from camera frames into servo angles:

```python
def mapping_value(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

Use case:

Maps cx from 0–640 → 0–180° for pan servo
Maps cy from 0–480 → 0–180° for tilt servo

This will be replaced with real servo movement via RPi.GPIO or pigpio later.

