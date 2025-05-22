#!/usr/bin/env python3
import time
import os
import sys

# ─────────────────────────────────────────────────────────────
# Add ONLY the path where Motor.py is located
# ─────────────────────────────────────────────────────────────
motor_path = os.path.expanduser('~/Shell-Eco-Marathon-2025/src/shared_objects/src/shared_objects')
if motor_path not in sys.path:
    sys.path.insert(0, motor_path)

# Correct import: not from shared_objects, but directly from Motor
from Motor import Stepper

# Optional: verify the file loaded
import inspect
print("Stepper definition from:", inspect.getfile(Stepper))

# Module IDs
modules = {
    "steering": 3,
    "brake":    1
}

# Create instance with positional arguments
brake_motor = Stepper(
    "can",              # interface
    1_000_000,          # data_rate
    modules["brake"],   # module_id
    50_000,             # max_velocity
    140_000,            # max_acc
    140_000,            # MaxDeceleration
    0,                  # V1
    50_000,             # A1
    50_000              # D1
)

# Execute brake
if __name__ == "__main__":
    try:
        print("Brake test starts in 3 seconds…")
        time.sleep(3)
        brake_motor.brake()
        print("Brake sequence completed successfully.")
    except Exception as e:
        print(f"[ERROR] {e}")
    finally:
        brake_motor.disconnect_motor()
        print("Motor disconnected. Test complete.")
