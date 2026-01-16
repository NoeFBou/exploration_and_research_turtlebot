#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import Bool, String, Int16MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class CatchNode(Node):
    """
    Pilotage pince.

    Entrées:
      - /catch (std_msgs/Bool) : True=close, False=open
      - /catch_cmd (std_msgs/String) OPTIONNEL : "open"/"close"/"toggle"
        -> IMPORTANT: c'est un AUTRE topic pour éviter le conflit de types.

    Modes:
      - sim  : publie JointTrajectory sur /gripper_controller/joint_trajectory
      - real : publie Int16MultiArray sur /servo_command (16 canaux), avec gripper ch12 + support ch13
    """

    def __init__(self):
        super().__init__("catch_node")

        # -------------------------
        # Paramètres
        # -------------------------
        self.declare_parameter("mode", "sim")  # 'sim' | 'real' (évite auto ambigu)
        #self.declare_parameter("use_sim_time", False)

        # Topics d'entrée
        self.declare_parameter("catch_topic", "/catch")         # Bool
        self.declare_parameter("catch_cmd_topic", "/catch_cmd") # String optionnel (mets "" pour désactiver)

        # SIM
        self.declare_parameter("traj_topic", "/gripper_controller/joint_trajectory")
        self.declare_parameter("left_joint", "gripper_left_joint")
        self.declare_parameter("right_joint", "gripper_right_joint")
        self.declare_parameter("sim_open_left", 0.9)
        self.declare_parameter("sim_open_right", -0.9)

        self.declare_parameter("sim_close_left", 0.0)
        self.declare_parameter("sim_close_right", 0.0)
        self.declare_parameter("sim_motion_time", 0.6)  # seconds

        # REAL (compat /servo_command)
        self.declare_parameter("servo_topic", "/servo_command")
        self.declare_parameter("servo_channels", 16)
        self.declare_parameter("gripper_channel", 12)
        self.declare_parameter("support_channel", 13)
        self.declare_parameter("real_open_angle", 0)      # deg
        self.declare_parameter("real_close_angle", 45)    # deg
        self.declare_parameter("real_support_angle", 0)   # deg

        self.mode = str(self.get_parameter("mode").value).strip().lower()
        if self.mode not in ("sim", "real"):
            raise RuntimeError("mode must be 'sim' or 'real'")

        self.state_closed = False
        self._last_sent_closed = None  # pour éviter spam
        self._init_open_sent = False
        self._init_log_once = False

        # -------------------------
        # Publishers
        # -------------------------
        if self.mode == "sim":
            self.traj_pub = self.create_publisher(
                JointTrajectory,
                self.get_parameter("traj_topic").value,
                10
            )
            self.get_logger().info(f"[SIM] Publishing JointTrajectory to {self.get_parameter('traj_topic').value}")
        else:
            self.servo_pub = self.create_publisher(
                Int16MultiArray,
                self.get_parameter("servo_topic").value,
                10
            )
            self.get_logger().info(f"[REAL] Publishing Int16MultiArray to {self.get_parameter('servo_topic').value}")

        # -------------------------
        # Subscriptions (IMPORTANT: /catch est Bool UNIQUEMENT)
        # -------------------------
        catch_topic = self.get_parameter("catch_topic").value
        self.create_subscription(Bool, catch_topic, self._on_bool, 10)

        cmd_topic = str(self.get_parameter("catch_cmd_topic").value).strip()
        if cmd_topic:
            self.create_subscription(String, cmd_topic, self._on_string, 10)
            self.get_logger().info(f"Also listening text commands on {cmd_topic} (String)")
        else:
            self.get_logger().info("Text commands disabled (catch_cmd_topic='')")

        # Init: ouvrir (robuste)
        self._init_timer = self.create_timer(0.2, self._try_send_initial_open)

        self.get_logger().info(f"catch_node started in mode='{self.mode}'. Subscribed to {catch_topic}")

    # -------------------------
    # Callbacks
    # -------------------------
    def _on_bool(self, msg: Bool):
        if msg.data:
            self.close_gripper()
        else:
            self.open_gripper()

    def _on_string(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd in ("close", "grasp", "catch", "1", "true"):
            self.close_gripper()
        elif cmd in ("open", "release", "drop", "0", "false"):
            self.open_gripper()
        elif cmd == "toggle":
            self.close_gripper() if not self.state_closed else self.open_gripper()
        else:
            self.get_logger().warn(f"Unknown command on /catch_cmd: '{msg.data}'")

    # -------------------------
    # API
    # -------------------------
    def open_gripper(self):
        self.state_closed = False
        self._send_command(closed=False)

    def close_gripper(self):
        self.state_closed = True
        self._send_command(closed=True)

    # -------------------------
    # Robust init open (évite message perdu au boot)
    # -------------------------
    def _try_send_initial_open(self):
        # On attend qu'il y ait au moins 1 subscriber côté commande
        if self.mode == "sim":
            subs = self.traj_pub.get_subscription_count()
            if subs == 0:
                if not self._init_log_once:
                    self.get_logger().warn("[SIM] Waiting for gripper_controller subscription...")
                    self._init_log_once = True
                return
        else:
            subs = self.servo_pub.get_subscription_count()
            if subs == 0:
                if not self._init_log_once:
                    self.get_logger().warn("[REAL] Waiting for servo_node subscription...")
                    self._init_log_once = True
                return

        # Dès que prêt: envoyer OPEN une fois et arrêter le timer
        if not self._init_open_sent:
            self.get_logger().info("Initial OPEN sent (controller/servo ready).")
            self.open_gripper()
            self._init_open_sent = True
            self._init_timer.cancel()

    # -------------------------
    # Command send
    # -------------------------
    def _send_command(self, closed: bool):
        if self._last_sent_closed == closed:
            return  # évite spam identique
        self._last_sent_closed = closed

        if self.mode == "sim":
            self._send_sim(closed)
        else:
            self._send_real(closed)

    def _send_sim(self, closed: bool):
        left_joint = self.get_parameter("left_joint").value
        right_joint = self.get_parameter("right_joint").value

        if closed:
            left = float(self.get_parameter("sim_close_left").value)
            right = float(self.get_parameter("sim_close_right").value)
            label = "CLOSE"
        else:
            left = float(self.get_parameter("sim_open_left").value)
            right = float(self.get_parameter("sim_open_right").value)
            label = "OPEN"

        msg = JointTrajectory()
        msg.joint_names = [left_joint, right_joint]

        p = JointTrajectoryPoint()
        p.positions = [left, right]
        p.time_from_start = Duration(seconds=float(self.get_parameter("sim_motion_time").value)).to_msg()
        msg.points = [p]

        self.traj_pub.publish(msg)
        self.get_logger().info(f"Gripper command (sim): {label} [{left:.2f}, {right:.2f}]")

    def _send_real(self, closed: bool):
        n = int(self.get_parameter("servo_channels").value)
        gripper_ch = int(self.get_parameter("gripper_channel").value)
        support_ch = int(self.get_parameter("support_channel").value)

        open_a = int(self.get_parameter("real_open_angle").value)
        close_a = int(self.get_parameter("real_close_angle").value)
        support_a = int(self.get_parameter("real_support_angle").value)

        angles = [0] * n
        angles[support_ch] = support_a
        angles[gripper_ch] = close_a if closed else open_a

        out = Int16MultiArray()
        out.data = angles
        self.servo_pub.publish(out)

        self.get_logger().info(
            f"Gripper command (real): {'CLOSE' if closed else 'OPEN'} "
            f"(ch{gripper_ch}={angles[gripper_ch]} deg, ch{support_ch}={angles[support_ch]} deg)"
        )


def main(args=None):
    rclpy.init(args=args)
    node = CatchNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
