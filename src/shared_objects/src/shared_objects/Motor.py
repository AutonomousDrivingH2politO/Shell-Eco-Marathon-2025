#!/usr/bin/env python3
"""
Stepper class for TMCM-1260 (Trinamic) with:
  • explicit initial zeroing of ActualPosition and EncoderPosition
  • continuous tracking of virtual (commanded) and effective (encoder) positions
  • slip detection / auto-realign
  • non-blocking brake routine (no sleep calls)

Author: <your-name> — 2025-05-15
"""
from time import time
from pytrinamic.connections import ConnectionManager
from pytrinamic.modules import TMCM1260

# global used only to keep the initial offset between virtual and effective positions
start_deviation_error = 0


def unwrap_position_signed(current_position: int, previous_position: int, max_position: int = 2 ** 32) -> int:
    """
    Unwrap a signed position so that rollover of the 32-bit counter does not break continuity.
    """
    # Convert the two readings to signed values
    current_signed = current_position - max_position if current_position >= max_position // 2 else current_position
    previous_signed = previous_position - max_position if previous_position >= max_position // 2 else previous_position

    diff = current_signed - previous_signed
    # Detect wrap-around
    if diff > max_position // 2:
        return previous_signed - (max_position - diff)
    elif diff < -max_position // 2:
        return previous_signed + (max_position + diff)
    return current_signed


class Stepper:
    """
    High-level wrapper for a single TMCM-1260 motor (axis 0).
    """

    def __init__(
        self,
        interface: str,
        data_rate: int,
        module_id: int,
        max_velocity: int,
        max_acc: int,
        max_deceleration: int,
        v1: int,
        a1: int,
        d1: int,
    ):
        global start_deviation_error

        # ------------------------- connection -------------------------------- #
        if interface == "can":
            transport = "socketcan_tmcl"
        elif interface == "usb":
            transport = "usb_tmcl"
        else:
            raise ValueError("interface must be 'can' or 'usb'")

        self.interface = ConnectionManager(
            f"--interface {transport} --port can1 --data-rate {data_rate}"
        ).connect()
        self.module = TMCM1260(self.interface, module_id=module_id)
        self.motor = self.module.motors[0]  # axis 0

        # ------------------------- motor setup -------------------------------- #
        # Motion parameters
        self.motor.set_axis_parameter(self.motor.AP.MaxVelocity, max_velocity)
        self.motor.set_axis_parameter(self.motor.AP.MaxAcceleration, max_acc)
        self.motor.set_axis_parameter(self.motor.AP.MaxDeceleration, max_deceleration)
        self.motor.set_axis_parameter(self.motor.AP.V1, v1)
        self.motor.set_axis_parameter(self.motor.AP.A1, a1)
        self.motor.set_axis_parameter(self.motor.AP.D1, d1)
        # Miscellaneous limits / current / smart energy
        self.motor.set_axis_parameter(self.motor.AP.StartVelocity, 1_000)
        self.motor.set_axis_parameter(self.motor.AP.StopVelocity, 1_000)
        self.motor.set_axis_parameter(self.motor.AP.RampWaitTime, 0)
        self.motor.set_axis_parameter(self.motor.AP.MaxCurrent, 200)
        self.motor.set_axis_parameter(self.motor.AP.StandbyCurrent, 100)
        self.motor.set_axis_parameter(self.motor.AP.SG2Threshold, 11)
        self.motor.set_axis_parameter(self.motor.AP.SG2FilterEnable, 0)
        self.motor.set_axis_parameter(self.motor.AP.SmartEnergyStallVelocity, 0)
        self.motor.set_axis_parameter(self.motor.AP.SmartEnergyHysteresis, 15)
        self.motor.set_axis_parameter(self.motor.AP.SmartEnergyHysteresisStart, 0)
        self.motor.set_axis_parameter(self.motor.AP.SECUS, 1)
        self.motor.set_axis_parameter(self.motor.AP.SmartEnergyThresholdSpeed, 7_999_774)

        # Driver settings
        self.motor.drive_settings.boost_current = 0
        self.motor.drive_settings.microstep_resolution = (
            self.motor.ENUM.MicrostepResolution256Microsteps
        )

        # ------------------------- zero positions ----------------------------- #
        # Reset BOTH actual (axis register 1) and encoder (axis register 209) positions
        self.motor.set_actual_position(0)  # wrapper for AP 1
        self.motor.set_axis_parameter(self.motor.AP.EncoderPosition, 0)

        # Tracking variables
        self.virtual_position = 0                    # commanded target
        self.effective_position = 0                  # encoder reading
        start_deviation_error = 0                    # no initial offset now

        # Unwrap helper
        self.last_encoder_position = 0
        self.unwrapped_position = 0

        print(f"[Stepper] Initialized on port can1 id={module_id}")

    # --------------------------------------------------------------------- #
    # helpers
    # --------------------------------------------------------------------- #
    def update_effective_position(self):
        """Refresh encoder position cache."""
        self.effective_position = self.motor.get_axis_parameter(
            self.motor.AP.EncoderPosition
        )

    def update_unwrapped_position(self):
        """Return continuous encoder position, compensating roll-over."""
        current_enc = self.motor.get_axis_parameter(self.motor.AP.EncoderPosition)
        self.unwrapped_position = unwrap_position_signed(
            current_enc, self.last_encoder_position
        )
        self.last_encoder_position = current_enc
        return self.unwrapped_position

    def compute_slip_error(self, threshold: int = 200):
        """
        Compare effective (encoder) and virtual (commanded) positions, returning
        (slipped?, correction_value).
        """
        self.update_effective_position()
        deviation_error = self.effective_position - self.virtual_position
        if abs(deviation_error) > threshold:
            return True, deviation_error
        return False, 0

    # --------------------------------------------------------------------- #
    # motion
    # --------------------------------------------------------------------- #
    def move_to(self, target: int):
        """Absolute move; keeps virtual_position in sync."""
        self.motor.move_to(target)
        self.virtual_position = target

    def move_by(self, delta: int):
        """Relative move; keeps virtual_position in sync."""
        self.motor.move_by(delta)
        self.virtual_position += delta

    # --------------------------------------------------------------------- #
    # non-blocking brake routine
    # --------------------------------------------------------------------- #
    def brake(self, retract_ticks: int = 70_000, step: int = -10, timeout_s: int = 10):
        """
        Progressive reverse movement (e.g., press brake) without sleeps.
        Pulls back `retract_ticks`, checking for slip and correcting on-the-fly.
        """
        print("[Stepper] Braking...")
        start_t = time()

        # 1. Ramp back rapidly to main brake position
        start_enc = self.motor.get_axis_parameter(self.motor.AP.EncoderPosition)
        end_target = start_enc - retract_ticks
        self.move_to(end_target)

        # 2. Fine approach with small relative steps while monitoring slip
        while time() - start_t < timeout_s:
            slipped, corr = self.compute_slip_error()
            if slipped:
                print(f"[WARN] slip detected → correcting by {corr} ticks")
                self.move_by(corr)  # immediately correct

            self.move_by(step)      # small incremental pull
            # Busy-wait ~3 ms between iterations using motor status polling
            _ = self.motor.get_status_word()
            if time() - start_t >= timeout_s:
                break

        # 3. Return to balanced position (align AP vs encoder)
        self.update_effective_position()
        diff = self.virtual_position - self.effective_position
        if diff != 0:
            self.move_by(diff)
        print("[Stepper] Brake sequence completed")

    # --------------------------------------------------------------------- #
    # housekeeping
    # --------------------------------------------------------------------- #
    def disconnect_motor(self):
        self.motor.stop()
        self.interface.close()
        print("[Stepper] Disconnected")
