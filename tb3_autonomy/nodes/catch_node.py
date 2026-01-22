#!/usr/bin/env python3
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import Bool, String, Int16MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class CatchNode(Node):
    """
    Node responsible for controlling the robotic gripper.

    Supports two modes:
    - 'sim': Publishes JointTrajectory messages for Gazebo simulation.
    - 'real': Publishes Int16MultiArray messages for the real robot's servo controller.

    Subscriptions:
      - /catch (std_msgs/Bool): True to close, False to open.
      - /catch_cmd (std_msgs/String): Optional text commands ("open", "close", "toggle").
    """

    def __init__(self):
        super().__init__("catch_node")

        # ---------------------------------------------------------------------
        # Parameters Declaration
        # ---------------------------------------------------------------------
        self.declare_parameter("mode", "sim")  # 'sim' or 'real'
        self.declare_parameter("catch_topic", "/catch")
        self.declare_parameter("catch_cmd_topic", "/catch_cmd")

        # Simulation Parameters
        self.declare_parameter("traj_topic", "/gripper_controller/joint_trajectory")
        self.declare_parameter("left_joint", "gripper_left_joint")
        self.declare_parameter("right_joint", "gripper_right_joint")
        self.declare_parameter("sim_open_left", 0.9)
        self.declare_parameter("sim_open_right", -0.9)
        self.declare_parameter("sim_close_left", 0.6)
        self.declare_parameter("sim_close_right", -0.6)
        self.declare_parameter("sim_motion_time", 0.6)  # seconds

        # Real Robot Parameters (Servo Controller)
        self.declare_parameter("servo_topic", "/servo_command")
        self.declare_parameter("servo_channels", 16)
        self.declare_parameter("gripper_channel", 12)
        self.declare_parameter("support_channel", 13)
        self.declare_parameter("real_open_angle", 0)    # degrees
        self.declare_parameter("real_close_angle", 45)  # degrees
        self.declare_parameter("real_support_angle", 0) # degrees

        # ---------------------------------------------------------------------
        # Initialization
        # ---------------------------------------------------------------------
        self.mode = str(self.get_parameter("mode").value).strip().lower()
        if self.mode not in ("sim", "real"):
            raise RuntimeError(f"Invalid mode '{self.mode}'. Must be 'sim' or 'real'.")

        self.state_closed = False
        self._last_sent_closed: Optional[bool] = None
        self._init_open_sent = False
        self._init_log_once = False

        # ---------------------------------------------------------------------
        # Publishers
        # ---------------------------------------------------------------------
        if self.mode == "sim":
            traj_topic = self.get_parameter("traj_topic").value
            self.traj_pub = self.create_publisher(JointTrajectory, traj_topic, 10)
            self.get_logger().info(f"[SIM] Initialized in SIMULATION mode. Topic: {traj_topic}")
        else:
            servo_topic = self.get_parameter("servo_topic").value
            self.servo_pub = self.create_publisher(Int16MultiArray, servo_topic, 10)
            self.get_logger().info(f"[REAL] Initialized in REAL ROBOT mode. Topic: {servo_topic}")

        # ---------------------------------------------------------------------
        # Subscribers
        # ---------------------------------------------------------------------
        catch_topic = self.get_parameter("catch_topic").value
        self.create_subscription(Bool, catch_topic, self._on_bool, 10)

        cmd_topic = str(self.get_parameter("catch_cmd_topic").value).strip()
        if cmd_topic:
            self.create_subscription(String, cmd_topic, self._on_string, 10)
            self.get_logger().info(f"Text command interface enabled on: {cmd_topic}")
        else:
            self.get_logger().info("Text command interface disabled.")

        # Robust initialization: wait for subscriber then open gripper
        self._init_timer = self.create_timer(0.2, self._try_send_initial_open)

    # -------------------------------------------------------------------------
    # Callbacks
    # -------------------------------------------------------------------------
    def _on_bool(self, msg: Bool):
        """Callback for boolean commands (True=Close, False=Open)."""
        if msg.data:
            self.close_gripper()
        else:
            self.open_gripper()

    def _on_string(self, msg: String):
        """Callback for string commands (open/close/toggle)."""
        cmd = msg.data.strip().lower()
        if cmd in ("close", "grasp", "catch", "1", "true"):
            self.close_gripper()
        elif cmd in ("open", "release", "drop", "0", "false"):
            self.open_gripper()
        elif cmd == "toggle":
            if not self.state_closed:
                self.close_gripper()
            else:
                self.open_gripper()
        else:
            self.get_logger().warn(f"Unknown command received: '{msg.data}'")

    # -------------------------------------------------------------------------
    # Public API
    # -------------------------------------------------------------------------
    def open_gripper(self):
        """Request gripper opening."""
        self.state_closed = False
        self._send_command(closed=False)

    def close_gripper(self):
        """Request gripper closing."""
        self.state_closed = True
        self._send_command(closed=True)

    # -------------------------------------------------------------------------
    # Internal Logic
    # -------------------------------------------------------------------------
    def _try_send_initial_open(self):
        """
        Periodically checks if the output topic has subscribers before sending
        the initial 'OPEN' command. This prevents the command from being lost at startup.
        """
        if self.mode == "sim":
            subs = self.traj_pub.get_subscription_count()
            log_prefix = "[SIM]"
        else:
            subs = self.servo_pub.get_subscription_count()
            log_prefix = "[REAL]"

        if subs == 0:
            if not self._init_log_once:
                self.get_logger().warn(f"{log_prefix} Waiting for controller subscription...")
                self._init_log_once = True
            return

        # Subscriber connected, send open command once
        if not self._init_open_sent:
            self.get_logger().info(f"{log_prefix} Controller connected. Sending initial OPEN.")
            self.open_gripper()
            self._init_open_sent = True
            self._init_timer.cancel()

    def _send_command(self, closed: bool):
        """Dispatches the command to the appropriate handler (Sim or Real)."""
        # Avoid spamming duplicate commands
        if self._last_sent_closed == closed:
            return
        self._last_sent_closed = closed

        if self.mode == "sim":
            self._send_sim(closed)
        else:
            self._send_real(closed)

    def _send_sim(self, closed: bool):
        """Publishes JointTrajectory for Gazebo."""
        left_joint = self.get_parameter("left_joint").value
        right_joint = self.get_parameter("right_joint").value

        if closed:
            left = float(self.get_parameter("sim_close_left").value)
            right = float(self.get_parameter("sim_close_right").value)
            action = "CLOSE"
        else:
            left = float(self.get_parameter("sim_open_left").value)
            right = float(self.get_parameter("sim_open_right").value)
            action = "OPEN"

        msg = JointTrajectory()
        msg.joint_names = [left_joint, right_joint]

        point = JointTrajectoryPoint()
        point.positions = [left, right]
        point.time_from_start = Duration(seconds=float(self.get_parameter("sim_motion_time").value)).to_msg()
        msg.points = [point]

        self.traj_pub.publish(msg)
        self.get_logger().info(f"[SIM] Gripper {action} -> Left: {left:.2f}, Right: {right:.2f}")

    def _send_real(self, closed: bool):
        """Publishes servo angles for the real robot."""
        num_channels = int(self.get_parameter("servo_channels").value)
        gripper_ch = int(self.get_parameter("gripper_channel").value)
        support_ch = int(self.get_parameter("support_channel").value)

        open_deg = int(self.get_parameter("real_open_angle").value)
        close_deg = int(self.get_parameter("real_close_angle").value)
        support_deg = int(self.get_parameter("real_support_angle").value)

        target_angle = close_deg if closed else open_deg

        # Prepare the 16-channel array
        angles = [0] * num_channels
        angles[support_ch] = support_deg
        angles[gripper_ch] = target_angle

        msg = Int16MultiArray()
        msg.data = angles
        self.servo_pub.publish(msg)

        action = "CLOSE" if closed else "OPEN"
        self.get_logger().info(f"[REAL] Gripper {action} -> Ch{gripper_ch}: {target_angle}°")


def main(args=None):
    rclpy.init(args=args)
    node = CatchNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()