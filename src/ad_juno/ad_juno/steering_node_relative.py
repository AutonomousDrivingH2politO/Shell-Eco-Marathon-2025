#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from std_msgs.msg import Float64
from shared_objects.Motor_relative import Stepper
from shared_objects.ROS_utils import Topics
import time

# Conversion constants
TRANSMISSION_RATIO = 15       # Gear ratio
MAX_ANGLE = 14                # Mechanical stop in degrees
STEP_ANGLE = 1.8              # Degrees per full step
MAX_DELTA = 2000              # Max microsteps per PID cycle
REALIGN_INTERVAL = 2        # Seconds between absolute realign

class PIDController:
    def __init__(self, kp, ki, kd):
        # Proportional, integral, derivative gains
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.last_error = 0.0

    def compute(self, target, actual, dt):
        """
        Compute the controller output as a relative delta.
        Here we use target=0 and actual=position_error, so the
        return value is "how much to move by" next.
        """
        error = target - actual
        self.integral += error * dt
        derivative = (error - self.last_error) / dt if dt > 0 else 0.0
        self.last_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

class SteeringNode(Node):
    def __init__(self):
        super().__init__('steering_node')
        self.get_logger().info("----RELATIVE Steering node has been started----")

        # Create the filtered‐encoder stepper
        self.steering = Stepper(
            interface="can",
            data_rate=1000000,
            module_id=3,
            max_velocity=80_000,
            max_acc=50_000,
            max_deceleration=50_000,
            v1=100_000,
            a1=50_000,
            d1=50_000,
            filter_window_size=4
        )

        # PID that outputs a relative step delta
        self.pid = PIDController(kp=1.0, ki=0.0, kd=0.0)
        self.latest_angle = 0.0

        # Subscribe to the steering angle topic
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        topics = Topics().topic_names
        self.create_subscription(Float64, topics["steering"],
                                self.callback_stepangle, qos)

        # Track when we last did an absolute realign
        self.last_realign = time.time()

        # Run the PID loop at 100 Hz (non‑blocking via ROS timer)
        self.create_timer(0.01, self.pid_loop)
        self.get_logger().info("Steering node @100 Hz with relative‐move and periodic realign initialized")

    def callback_stepangle(self, msg: Float64):
        """
        Update the target angle (in degrees) when a new message arrives.
        """
        self.latest_angle = msg.data
        self.get_logger().info(f"New target angle: {self.latest_angle:.2f}")

    def convert_angle_to_position(self, angle: float) -> int:
        """
        Convert degrees to microsteps, including gear ratio.
        """
        steps = (angle / STEP_ANGLE) * 256
        return -int(steps * TRANSMISSION_RATIO)

    def pid_loop(self):
        """
        Runs every 0.01 s:
         1. Read filtered, unwrapped position
         2. Compute angle target → absolute microstep target
         3. Compute error and PID output as a relative delta
         4. Clamp and send move_by(delta)
         5. Every REALIGN_INTERVAL send move_to(target) to correct drift
        """
        now = time.time()
        dt = 0.01

        # 1) get filtered actual position
        actual_pos = self.steering.update_unwrapped_position()

        # 2) clamp target angle, compute absolute target steps
        tgt_deg = max(-MAX_ANGLE, min(self.latest_angle, MAX_ANGLE))
        abs_target = self.convert_angle_to_position(tgt_deg)

        # 3) error in microsteps
        error_steps = abs_target - actual_pos

        # 4) PID incremental: target=0, actual=error_steps
        delta = self.pid.compute(0, error_steps, dt)
        delta_int = int(delta)

        # 5) clamp the step delta
        delta_int = max(-MAX_DELTA, min(MAX_DELTA, delta_int))
        self.steering.move_stepper(delta_int)

        # 6) periodic absolute realignment
        if (now - self.last_realign) >= REALIGN_INTERVAL:
            self.steering.move_to_position(abs_target)
            self.last_realign = now

        self.get_logger().debug(
            f"err={error_steps:.1f}  delta={delta_int}"
            + ("  [realigned]" if now == self.last_realign else "")
        )

    def destroy_node(self):
        """
        Ensure the motor is stopped and interface closed.
        """
        self.steering.disconnect_motor()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SteeringNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("User interrupt, shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
