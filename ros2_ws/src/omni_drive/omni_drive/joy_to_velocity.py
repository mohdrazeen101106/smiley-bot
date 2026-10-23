"""Convert /joy into robot_interfaces/msg/Velocity for the unchanged Blitz sender."""

import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from robot_interfaces.msg import Velocity
from omni_drive.velocity_mapping import clamp, map_velocity, shape_axis

class JoyToVelocityNode(Node):
    def __init__(self) -> None:
        super().__init__("joy_to_velocity")
        defaults = {
            "joy_topic": "/joy",
            "velocity_topic": "/cmd_vel",
            "axis_forward": 1,
            "axis_lateral": 0,
            "forward_sign": -1.0,
            "lateral_sign": -1.0,
            # Button mode is the safe default because some D-pads report a
            # non-zero resting value in Joy.axes (which previously caused yaw).
            "yaw_input_mode": "buttons",  # 'buttons' or 'axis'
            "axis_yaw": 6,
            "yaw_sign": -1.0,
            "yaw_left_button": -1,   # Set to real buttons[] index; -1 disables
            "yaw_right_button": -1,  # Set to real buttons[] index; -1 disables
            "deadman_button": 4,
            "require_deadman": True,
            "deadzone": 0.05,
            "joy_timeout": 0.25,
            "publish_rate": 50.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        get = lambda name: self.get_parameter(name).value

        self.joy_topic = str(get("joy_topic"))
        self.velocity_topic = str(get("velocity_topic"))
        self.axis_forward = int(get("axis_forward"))
        self.axis_lateral = int(get("axis_lateral"))
        self.forward_sign = float(get("forward_sign"))
        self.lateral_sign = float(get("lateral_sign"))
        self.yaw_mode = str(get("yaw_input_mode")).lower()
        self.axis_yaw = int(get("axis_yaw"))
        self.yaw_sign = float(get("yaw_sign"))
        self.yaw_left_button = int(get("yaw_left_button"))
        self.yaw_right_button = int(get("yaw_right_button"))
        self.deadman_button = int(get("deadman_button"))
        self.require_deadman = bool(get("require_deadman"))
        self.deadzone = float(get("deadzone"))
        self.joy_timeout = float(get("joy_timeout"))
        self.publish_rate = float(get("publish_rate"))

        if self.yaw_mode not in ("buttons", "axis"):
            raise ValueError("yaw_input_mode must be 'buttons' or 'axis'")
        if not 0.0 <= self.deadzone < 1.0:
            raise ValueError("deadzone must be in [0, 1)")
        if self.joy_timeout <= 0 or self.publish_rate <= 0:
            raise ValueError("joy_timeout and publish_rate must be positive")
        for name, sign in (("forward_sign", self.forward_sign), ("lateral_sign", self.lateral_sign), ("yaw_sign", self.yaw_sign)):
            if not math.isfinite(sign) or sign not in (-1.0, 1.0):
                raise ValueError(f"{name} must be +1.0 or -1.0")

        self.latest_joy = None
        self.last_joy_monotonic = None
        self.velocity_pub = self.create_publisher(Velocity, self.velocity_topic, 10)
        self.joy_sub = self.create_subscription(Joy, self.joy_topic, self.on_joy, 10)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.control_tick)

        self.get_logger().info(
            f"Joy {self.joy_topic} -> Velocity {self.velocity_topic}; "
            f"deadman index={self.deadman_button}, yaw mode={self.yaw_mode}."
        )

    def on_joy(self, msg: Joy) -> None:
        self.latest_joy = msg
        self.last_joy_monotonic = time.monotonic()

    @staticmethod
    def read_axis(msg: Joy, index: int) -> float:
        if index < 0 or index >= len(msg.axes):
            return 0.0
        value = float(msg.axes[index])
        return clamp(value) if math.isfinite(value) else 0.0

    @staticmethod
    def button_pressed(msg: Joy, index: int) -> bool:
        return 0 <= index < len(msg.buttons) and msg.buttons[index] == 1

    def yaw_value(self, msg: Joy) -> float:
        if self.yaw_mode == "axis":
            # Return raw axis input; map_velocity applies the deadzone once.
            return self.read_axis(msg, self.axis_yaw)
        # Left = positive counterclockwise; right = negative clockwise.
        return float(self.button_pressed(msg, self.yaw_left_button)) - float(self.button_pressed(msg, self.yaw_right_button))

    def calculate_velocity(self) -> tuple[float, float, float]:
        fresh = (
            self.latest_joy is not None
            and self.last_joy_monotonic is not None
            and time.monotonic() - self.last_joy_monotonic <= self.joy_timeout
        )
        if not fresh:
            return 0.0, 0.0, 0.0

        msg = self.latest_joy
        if self.require_deadman and not self.button_pressed(msg, self.deadman_button):
            return 0.0, 0.0, 0.0

        yaw = self.yaw_value(msg)
        # In button mode yaw is already -1, 0, +1; don't invert it via an
        # axis sign meant for an analogue axis.
        if self.yaw_mode == "buttons":
            return map_velocity(
                self.read_axis(msg, self.axis_forward),
                self.read_axis(msg, self.axis_lateral),
                yaw,
                deadzone=self.deadzone,
                forward_sign=self.forward_sign,
                lateral_sign=self.lateral_sign,
                yaw_sign=1.0,
            )
        return map_velocity(
            self.read_axis(msg, self.axis_forward),
            self.read_axis(msg, self.axis_lateral),
            yaw,
            deadzone=self.deadzone,
            forward_sign=self.forward_sign,
            lateral_sign=self.lateral_sign,
            yaw_sign=self.yaw_sign,
        )

    def control_tick(self) -> None:
        vx, vy, vw = self.calculate_velocity()
        msg = Velocity()
        msg.vx = float(vx)
        msg.vy = float(vy)
        msg.vw = float(vw)
        self.velocity_pub.publish(msg)

    def stop(self) -> None:
        msg = Velocity()
        msg.vx = msg.vy = msg.vw = 0.0
        for _ in range(3):
            try:
                self.velocity_pub.publish(msg)
            except Exception:
                break


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = JoyToVelocityNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
