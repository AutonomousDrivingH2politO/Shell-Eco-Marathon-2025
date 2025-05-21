#!/usr/bin/env python3
"""
Throttle node with BOTH:
  • Speed-hysteresis control (open under `open_thresh`, close above `close_thresh`)
  • Safety timeout (force-close if valve stays open longer than `open_timeout_sec`)
  • Optimised QoS settings (depth = 1 everywhere, BEST_EFFORT on high-rate sensors)

Topics (adapt to your shared_objects.ROS_utils.Topics mapping):
  ­ speed            (std_msgs/Float32) : current vehicle speed  [km/h]
  ­ requested_speed  (std_msgs/Float32) : live update of open_thresh
  ­ throttle         (std_msgs/Float32) : command to intake valve (50 = open, 0 = closed)
  ­ stop             (std_msgs/Bool)    : external emergency stop
  ­ model_enable     (std_msgs/Bool)    : perception / segmentation ready
  ­ engine_enable    (std_msgs/Bool)    : ECU says ICE is allowed to run
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, Bool
from shared_objects.ROS_utils import Topics


class ThrottleSafeHyst(Node):
    """ROS 2 node that opens the intake valve when speed is low and all enables are true,
    with hysteresis to avoid chattering and a watchdog timeout to prevent runaway open."""

    # --------------------------------------------------
    # Constructor
    # --------------------------------------------------
    def __init__(self):
        super().__init__("throttle_node")

        # ---------- Tunable parameters -----------------
        self.open_thresh      = 5.0      # km/h : open when speed ≤ open_thresh
        self.close_thresh     = 6.0      # km/h : close when speed ≥ close_thresh
        self.throttle_cmd_val = 50.0     # value sent when valve is open
        self.open_timeout_sec = 6.5      # safety watchdog (seconds)
        self.check_period     = 0.05     # watchdog / housekeeping period (s)
        # ------------------------------------------------

        # ---------------- Internal state ---------------
        self._stop          = False
        self._model_enable  = False
        self._engine_enable = True
        self._valve_open    = False
        self._last_open_ts  = 0.0
        # ------------------------------------------------

        # -------- QoS profiles (depth = 1) -------------
        cmd_qos   = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        speed_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        flag_qos  = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        # ------------------------------------------------

        # -------------- Publishers & subs --------------
        tn = Topics().topic_names
        self.throttle_pub = self.create_publisher(Float32, tn["throttle"], cmd_qos)

        self.create_subscription(Float32, tn["speed"],           self.cb_speed,           speed_qos)
        self.create_subscription(Float32, tn["requested_speed"], self.cb_requested_speed, flag_qos)
        self.create_subscription(Bool,    tn["stop"],            self.cb_stop,            flag_qos)
        self.create_subscription(Bool,    tn["model_enable"],    self.cb_model_enable,    flag_qos)
        self.create_subscription(Bool,    tn["engine_enable"],   self.cb_engine_enable,   flag_qos)
        # ------------------------------------------------

        # Periodic timer to enforce timeout even if no new speed arrives
        self.create_timer(self.check_period, self.watchdog)

        self.get_logger().info("ThrottleSafeHyst node ready")

    # ==================================================
    # Subscription callbacks (update internal state)
    # ==================================================
    def cb_stop(self, msg: Bool):
        self._stop = msg.data

    def cb_model_enable(self, msg: Bool):
        self._model_enable = msg.data

    def cb_engine_enable(self, msg: Bool):
        self._engine_enable = msg.data

    def cb_requested_speed(self, msg: Float32):
        """Allow runtime update of the lower threshold; keep the gap constant."""
        self.open_thresh = float(msg.data)
        # keep close_thresh ≥ open_thresh + 1 km/h
        self.close_thresh = max(self.open_thresh + 1.0, self.close_thresh)
        self.get_logger().info(f"Updated thresholds: open ≤ {self.open_thresh:.2f}  "
                               f"close ≥ {self.close_thresh:.2f}")

    # ==================================================
    # Core logic triggered by every speed sample
    # ==================================================
    def cb_speed(self, msg: Float32):
        speed = msg.data
        now   = time.time()

        # --------- Decide if we should open ----------
        should_open = (not self._valve_open and
                       not self._stop and self._model_enable and self._engine_enable and
                       speed <= self.open_thresh)

        if should_open:
            self._valve_open   = True
            self._last_open_ts = now
            self.publish_throttle(self.throttle_cmd_val)
            self.get_logger().debug(
                f"Valve OPEN  (speed={speed:.2f} ≤ {self.open_thresh})")
            return  # early exit

        # --------- Decide if we should close ----------
        should_close = (self._valve_open and
                        (speed >= self.close_thresh or
                         self._stop or
                         not self._model_enable or
                         not self._engine_enable))

        if should_close:
            self._valve_open = False
            self.publish_throttle(0.0)
            self.get_logger().debug(
                f"Valve CLOSE (speed={speed:.2f} ≥ {self.close_thresh} "
                f"or disable flag)")
            return
        # Otherwise keep current state (inside hysteresis band).

    # ==================================================
    # Watchdog timer – closes valve after timeout
    # ==================================================
    def watchdog(self):
        if not self._valve_open:
            return                                   # closed, nothing to do
        if time.time() - self._last_open_ts >= self.open_timeout_sec:
            self._valve_open = False
            self.publish_throttle(0.0)
            self.get_logger().warn(
                f"Safety timeout ({self.open_timeout_sec}s) reached → valve forced CLOSED")

    # ==================================================
    # Helper: publish a brand-new message every time
    # ==================================================
    def publish_throttle(self, value: float):
        msg = Float32()
        msg.data = value
        self.throttle_pub.publish(msg)


# ------------------------------------------------------
# Standard ROS 2 node boilerplate
# ------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = ThrottleSafeHyst()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
