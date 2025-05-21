#!/usr/bin/env python3
# import RPi.GPIO as GPIO
import Jetson.GPIO as GPIO
import time
import os

# Define the GPIO pin to monitor (BCM numbering)
INPUT_PIN = 18  # Change this to the pin you're using

# Setup GPIO
GPIO.setwarnings(False)  # Disable warnings
GPIO.setmode(GPIO.BCM)   # Use BCM pin numbering
GPIO.setup(INPUT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)  # Set pin as input with pull-down resistor

print(f"Monitoring GPIO pin {INPUT_PIN} for 3.3V power...")
print("Press CTRL+C to exit")

try:
    last_state = GPIO.input(INPUT_PIN)
    print(f"Initial state: {'HIGH (3.3V)' if last_state else 'LOW (0V)'}")

    while True:
        # Read current pin state
        current_state = GPIO.input(INPUT_PIN)

        # If the state has changed
        if current_state != last_state:
            if current_state:

                print("POWER DETECTED! (3.3V on pin)\n")
                os.subproces()
                print("System would be ENGAGED")
            else:

                print("Power removed (0V on pin)\n")

                print("System would be DISENGAGED")

            # Update the last state
            last_state = current_state

        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nExiting program")
finally:
    GPIO.cleanup()  # Clean up GPIO on exit