#!/usr/bin/env python3
from pytrinamic.connections import ConnectionManager
from pytrinamic.modules import TMCM1260
from time import time

start_deviation_error = 0

def unwrap_position_signed(current_position, previous_position, max_position=2**32):
    """
    Unwrap a raw encoder count so that it increments/decrements continuously
    across the rollover point at max_position.
    """
    # Map both readings into signed range
    if current_position >= max_position // 2:
        current_signed = current_position - max_position
    else:
        current_signed = current_position

    if previous_position >= max_position // 2:
        previous_signed = previous_position - max_position
    else:
        previous_signed = previous_position

    # Compute difference with wrap handling
    diff = current_signed - previous_signed
    if diff > max_position // 2:
        return previous_signed - (max_position - diff)
    elif diff < -max_position // 2:
        return previous_signed + (max_position + diff)
    else:
        return current_signed


class Stepper:
    def __init__(self,
                 interface, data_rate, module_id,
                 max_velocity, max_acc, max_deceleration,
                 v1, a1, d1,
                 filter_window_size: int = 5):
        global start_deviation_error

        # Select CAN or USB transport
        if interface == "can":
            transport = "socketcan_tmcl"
        elif interface == "usb":
            transport = "usb_tmcl"
        else:
            raise ValueError(f"Unknown interface '{interface}'")

        # Establish connection and instantiate module
        conn_str = f"--interface {transport} --port can1 --data-rate {data_rate}"
        self.interface = ConnectionManager(conn_str).connect()
        self.module = TMCM1260(self.interface, module_id=module_id)
        self.motor = self.module.motors[0]

        # Configure motion parameters
        self.motor.set_axis_parameter(self.motor.AP.MaxVelocity,       max_velocity)
        self.motor.set_axis_parameter(self.motor.AP.MaxAcceleration,   max_acc)
        self.motor.set_axis_parameter(self.motor.AP.MaxDeceleration,   max_deceleration)
        self.motor.set_axis_parameter(self.motor.AP.V1,                v1)
        self.motor.set_axis_parameter(self.motor.AP.A1,                a1)
        self.motor.set_axis_parameter(self.motor.AP.D1,                d1)
        # … any other axis or drive settings …

        # Set to zero the encoder and compute start‐up offset
        self.virtual_position = self.motor.get_actual_position()
        self.motor.set_axis_parameter(self.motor.AP.EncoderPosition, 0)
        eff_pos = self.motor.get_axis_parameter(self.motor.AP.EncoderPosition)
        start_deviation_error = eff_pos - self.virtual_position

        # Prepare for unwrapping + moving‐average filter
        self.last_encoder = self.motor.get_axis_parameter(self.motor.AP.EncoderPosition)
        self.filter_window = filter_window_size
        self.buffer = []
        self.unwrapped_position = 0.0

    def update_unwrapped_position(self):
        """
        Read encoder, apply signed unwrap, then return a moving‐average
        filtered position to reduce noise before PID.
        """
        raw = self.motor.get_axis_parameter(self.motor.AP.EncoderPosition)
        signed = unwrap_position_signed(raw, self.last_encoder)
        self.last_encoder = raw

        # Push into circular buffer
        # It creates a buffer queue that is storing values changing the oldest value everytime
        self.buffer.append(signed)
        if len(self.buffer) > self.filter_window:
            self.buffer.pop(0)

        # Then it returns the average value inside the buffer queue
        # Having a large filter_window can lead to an uncorrect value of position
        self.unwrapped_position = sum(self.buffer) / len(self.buffer)
        return self.unwrapped_position

    def compute_error(self, threshold=200):
        """
        Check if encoder vs. actual deviance exceeds threshold.
        Returns (slip_detected: bool, deviation: int).
        """
        deviation = (self.motor.get_axis_parameter(self.motor.AP.EncoderPosition)
                     - self.virtual_position
                     - start_deviation_error)
        if abs(deviation) > threshold:
            return True, deviation
        else:
            return False, 0

    def brake(self, run_time=10.0, step_back= -10, slip_correction_sleep=2.0):
        """
        Perform a brake routine:
          1. Move back by a large fixed offset.
          2. Over 'run_time' seconds, repeatedly:
             - step back by 'step_back' microsteps
             - if slip detected, correct by moving the slip amount
          3. Return to near-zero stretch position.
        All blocking waits are done via timestamp checks to avoid sleep().
        """
        start_t = time()
        # 1) initial back‐off
        start_pos = self.motor.get_axis_parameter(self.motor.AP.EncoderPosition)
        end_pos   = start_pos - 70000
        self.motor.move_to(end_pos)

        # 2) main loop for slip correction
        last_correction_t = time()
        while (time() - start_t) < run_time:
            # relative step
            self.motor.move_by(step_back)

            # slip correction at intervals of slip_correction_sleep
            if (time() - last_correction_t) >= slip_correction_sleep:
                slipped, dev = self.compute_error()
                if slipped:
                    self.motor.move_by(dev)
                last_correction_t = time()
            # busy‐wait until next loop iteration
            # (no sleep call)

        # 3) return to near‐zero stretch
        curr_ap = self.motor.get_axis_parameter(self.motor.AP.ActualPosition)
        curr_enc = self.motor.get_axis_parameter(self.motor.AP.EncoderPosition)
        offset = curr_ap - curr_enc
        # move until encoder >= -100
        while curr_enc < -100:
            self.motor.move_to(offset)
            curr_enc = self.motor.get_axis_parameter(self.motor.AP.EncoderPosition)
        self.motor.move_to(offset)

    def move_stepper(self, delta_steps: int):
        """
        Issue a relative‐move command by `delta_steps` microsteps.
        This extends the active motion profile without restarting it.
        """
        print(f"[Stepper] relative move_by: {delta_steps}")
        self.motor.move_by(delta_steps)

    def move_to_position(self, abs_steps: int):
        """
        Issue an absolute‐move command to `abs_steps` microsteps.
        Used periodically to realign any lost increments.
        """
        print(f"[Stepper] absolute move_to: {abs_steps}")
        self.motor.move_to(abs_steps)

    def disconnect_motor(self):
        """
        Stop the motor and close the transport interface cleanly.
        """
        self.motor.stop()
        self.interface.close()
