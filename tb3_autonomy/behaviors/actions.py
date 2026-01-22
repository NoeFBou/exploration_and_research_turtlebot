#!/usr/bin/env python3
import math
import time
from typing import Optional

import rclpy
import py_trees
import tf2_ros
from tf2_geometry_msgs import do_transform_pose

from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool, Int32, String


# =============================================================================
# NAVIGATION & MOVEMENT BEHAVIORS
# =============================================================================

class RotateToTarget(py_trees.behaviour.Behaviour):
    """
    Rotates the robot in place to face the target pose stored in the blackboard.

    Uses tf2 to transform the target pose into the robot's base frame.
    """

    def __init__(self, name: str = "Alignement", threshold: float = 0.05):
        """
        Args:
            name (str): Name of the behavior.
            threshold (float): Angular error tolerance in radians.
        """
        super(RotateToTarget, self).__init__(name)
        self.threshold = threshold
        self.node: Optional[Node] = None
        self.cmd_vel_pub = None
        self.tf_buffer = None
        self.tf_listener = None

        self.blackboard = py_trees.blackboard.Client(name="Align")
        self.blackboard.register_key(key="target_pose_map", access=py_trees.common.Access.READ)

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

    def initialise(self):
        self.node.get_logger().info(f"[{self.name}] Starting rotation to target...")

    def update(self):
        target_map = self.blackboard.target_pose_map
        if target_map is None:
            return py_trees.common.Status.FAILURE

        try:
            # Look up transform at time 0 to avoid TF latency issues
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                'map',
                rclpy.time.Time(seconds=0)
            )
            target_local = do_transform_pose(target_map.pose, transform)
        except Exception:
            return py_trees.common.Status.RUNNING

        x = target_local.position.x
        y = target_local.position.y
        angle_error = math.atan2(y, x)

        # Check if aligned
        if abs(angle_error) < self.threshold:
            self.node.get_logger().info(f"[{self.name}] Target aligned. Stopping.")
            self.cmd_vel_pub.publish(Twist())
            return py_trees.common.Status.SUCCESS

        # P-Controller for rotation
        twist = Twist()
        twist.linear.x = 0.0
        k_p = 1.5
        max_speed = 0.5
        angular_speed = k_p * angle_error

        # Clamp speed
        twist.angular.z = max(min(angular_speed, max_speed), -max_speed)

        # Minimum speed to overcome friction
        if abs(twist.angular.z) < 0.1:
            twist.angular.z = 0.1 if angle_error > 0 else -0.1

        self.cmd_vel_pub.publish(twist)
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if self.cmd_vel_pub:
            self.cmd_vel_pub.publish(Twist())


class VisualServoingApproach(py_trees.behaviour.Behaviour):
    """
    Performs a visual servoing approach towards the target.
    Moves forward while correcting the heading until a minimum distance is reached.
    """

    def __init__(self, name: str = "Visual Servoing", min_dist: float = 0.18):
        """
        Args:
            name (str): Name of the behavior.
            min_dist (float): Stopping distance from the target in meters.
        """
        super(VisualServoingApproach, self).__init__(name)
        self.blackboard = py_trees.blackboard.Client(name="Action")
        self.blackboard.register_key(key="target_pose_map", access=py_trees.common.Access.READ)
        self.node: Optional[Node] = None
        self.cmd_vel_pub = None
        self.tf_buffer = None
        self.tf_listener = None
        self.target_min_dist = min_dist

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

    def initialise(self):
        self.node.get_logger().info(f"[{self.name}] Starting visual approach...")

    def update(self):
        target_map = self.blackboard.target_pose_map
        if target_map is None:
            return py_trees.common.Status.FAILURE

        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                'map',
                rclpy.time.Time(seconds=0)
            )
            target_local = do_transform_pose(target_map.pose, transform)
        except Exception:
            return py_trees.common.Status.RUNNING

        x_dist = target_local.position.x
        y_lat = target_local.position.y

        # Check arrival condition
        if x_dist <= self.target_min_dist:
            self.node.get_logger().info(f"[{self.name}] Target reached (Dist={x_dist:.2f}m).")
            self.cmd_vel_pub.publish(Twist())
            return py_trees.common.Status.SUCCESS

        # Simple Proportional Control
        twist = Twist()
        # Slow down when close
        twist.linear.x = 0.15 if x_dist > 0.4 else 0.05
        # Correct heading
        twist.angular.z = 0.8 * y_lat

        self.cmd_vel_pub.publish(twist)
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if new_status != py_trees.common.Status.RUNNING and self.cmd_vel_pub:
            self.cmd_vel_pub.publish(Twist())


class BackUp(py_trees.behaviour.Behaviour):
    """
    Moves the robot backward for a specified duration.
    """

    def __init__(self, name: str = "Back Up", duration: float = 2.0, speed: float = -0.1):
        super(BackUp, self).__init__(name)
        self.duration = duration
        self.speed = speed
        self.start_time = None
        self.node: Optional[Node] = None
        self.pub = None

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.pub = self.node.create_publisher(Twist, '/cmd_vel', 10)

    def initialise(self):
        self.start_time = self.node.get_clock().now()
        self.node.get_logger().info(f"[{self.name}] Backing up...")

    def update(self):
        now = self.node.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9

        if elapsed > self.duration:
            self.pub.publish(Twist())
            return py_trees.common.Status.SUCCESS

        msg = Twist()
        msg.linear.x = self.speed
        self.pub.publish(msg)
        return py_trees.common.Status.RUNNING


class ManualRecovery(py_trees.behaviour.Behaviour):
    """
    Activates manual control mode on the mission controller and waits for user confirmation.
    """

    def __init__(self, name: str = "Manual Recovery"):
        super(ManualRecovery, self).__init__(name)
        self.node: Optional[Node] = None
        self.pub_status = None
        self.pub_catch = None
        self.sub_conf = None
        self.confirmation = None

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.pub_status = self.node.create_publisher(String, '/mission/robot_status', 10)
        self.pub_catch = self.node.create_publisher(Bool, '/catch', 10)
        self.sub_conf = self.node.create_subscription(Bool, '/mission/confirmation', self._cb, 10)

    def initialise(self):
        self.confirmation = None
        self.node.get_logger().info(f"[{self.name}] Safety: Opening gripper.")
        self.pub_catch.publish(Bool(data=False))

        msg = String()
        msg.data = "MANUAL_RECOVERY"
        self.pub_status.publish(msg)
        self.node.get_logger().info(f"[{self.name}] Waiting for human operator...")

    def _cb(self, msg):
        self.confirmation = msg.data

    def update(self):
        if self.confirmation is None:
            return py_trees.common.Status.RUNNING

        if self.confirmation:
            self.node.get_logger().info(f"[{self.name}] Operator confirmed success.")
            return py_trees.common.Status.SUCCESS
        else:
            self.node.get_logger().info(f"[{self.name}] Operator reported failure.")
            return py_trees.common.Status.FAILURE


# =============================================================================
# LOGIC & SIGNAL BEHAVIORS
# =============================================================================

class PublishStatus(py_trees.behaviour.Behaviour):
    """
    Publishes a specific status string to the /mission/robot_status topic.
    Useful for updating the UI state (e.g., setting IDLE).
    """

    def __init__(self, name: str = "Publish Status", status: str = "IDLE"):
        super(PublishStatus, self).__init__(name)
        self.target_status = status
        self.node: Optional[Node] = None
        self.pub = None

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.pub = self.node.create_publisher(String, '/mission/robot_status', 10)

    def update(self):
        msg = String()
        msg.data = self.target_status
        self.pub.publish(msg)
        return py_trees.common.Status.SUCCESS


class WaitDuration(py_trees.behaviour.Behaviour):
    """
    Waits for a specific duration before returning SUCCESS.
    """

    def __init__(self, name: str = "Timer", duration: float = 100.0):
        super(WaitDuration, self).__init__(name)
        self.duration = duration
        self.start_time = None

    def initialise(self):
        self.start_time = time.time()

    def update(self):
        elapsed = time.time() - self.start_time
        if elapsed > self.duration:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class WaitForUserSelection(py_trees.behaviour.Behaviour):
    """
    Waits for the user to select a target ID via the mission controller.
    Updates the blackboard with the selected target's pose.
    """

    def __init__(self, name: str = "Wait Selection"):
        super(WaitForUserSelection, self).__init__(name)
        self.node: Optional[Node] = None
        self.sub = None
        self.pub_status = None
        self.received_id = -1

        self.blackboard = py_trees.blackboard.Client(name="Interface")
        self.blackboard.register_key(key="known_objects", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="target_pose_map", access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.sub = self.node.create_subscription(Int32, '/mission/select_target', self._msg_callback, 10)
        self.pub_status = self.node.create_publisher(String, '/mission/robot_status', 10)

    def _msg_callback(self, msg):
        self.node.get_logger().info(f"[UI] Received selection: ID {msg.data}")
        self.received_id = msg.data

    def initialise(self):
        self.node.get_logger().info("[UI] Waiting for user selection...")
        msg = String()
        msg.data = "IDLE"
        self.pub_status.publish(msg)

    def update(self):
        if self.received_id == -1:
            return py_trees.common.Status.RUNNING

        objects = self.blackboard.known_objects
        if not objects:
            return py_trees.common.Status.FAILURE

        target_id = self.received_id
        selected_obj = next((o for o in objects if o['id'] == target_id), None)

        if selected_obj:
            self.node.get_logger().info(f"[UI] Target #{target_id} confirmed.")
            self.blackboard.target_pose_map = selected_obj['pose']
            self.received_id = -1
            return py_trees.common.Status.SUCCESS
        else:
            self.node.get_logger().warn(f"[UI] ID {target_id} is invalid or unknown.")
            self.received_id = -1
            self.pub_status.publish(String(data="IDLE"))
            return py_trees.common.Status.RUNNING


class WaitForSkipSignal(py_trees.behaviour.Behaviour):
    """
    Monitors the skip signal topic. Returns SUCCESS if a skip is requested.
    """

    def __init__(self, name: str = "Wait Skip"):
        super(WaitForSkipSignal, self).__init__(name)
        self.node: Optional[Node] = None
        self.sub = None
        self.skip_received = False

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.sub = self.node.create_subscription(Bool, '/mission/skip_nav', self._cb, 10)

    def initialise(self):
        self.skip_received = False

    def _cb(self, msg):
        if self.status == py_trees.common.Status.RUNNING:
            if msg.data:
                self.node.get_logger().info(f"[{self.name}] SKIP signal received.")
                self.skip_received = True

    def update(self):
        if self.skip_received:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class WaitForAbortSignal(py_trees.behaviour.Behaviour):
    """
    Monitors the abort signal topic. Returns SUCCESS if an abort is requested.
    """

    def __init__(self, name: str = "Wait Abort"):
        super(WaitForAbortSignal, self).__init__(name)
        self.node: Optional[Node] = None
        self.sub = None
        self.abort_received = False

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.sub = self.node.create_subscription(Bool, '/mission/abort', self._cb, 10)

    def initialise(self):
        self.abort_received = False

    def _cb(self, msg):
        if self.status == py_trees.common.Status.RUNNING:
            if msg.data:
                self.node.get_logger().info(f"[{self.name}] ABORT signal received.")
                self.abort_received = True

    def update(self):
        if self.abort_received:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class WaitForStartSignal(py_trees.behaviour.Behaviour):
    """
    Waits for the start signal from the controller to begin the mission.
    """

    def __init__(self, name: str = "Wait Start"):
        super(WaitForStartSignal, self).__init__(name)
        self.node: Optional[Node] = None
        self.sub = None
        self.start_received = False

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.sub = self.node.create_subscription(Bool, '/mission/start', self._cb, 10)

    def initialise(self):
        self.start_received = False
        self.node.get_logger().info("[Supervisor] Waiting for START signal...")

    def _cb(self, msg):
        if msg.data:
            self.start_received = True

    def update(self):
        return py_trees.common.Status.SUCCESS if self.start_received else py_trees.common.Status.RUNNING


class WaitForConfirmation(py_trees.behaviour.Behaviour):
    """
    Asks for user confirmation via the mission controller.
    Publishes a status message and waits for a boolean response.
    """

    def __init__(self, name: str = "Human Validation", status_msg: str = "WAITING_CONFIRMATION"):
        super(WaitForConfirmation, self).__init__(name)
        self.node: Optional[Node] = None
        self.pub_status = None
        self.sub_conf = None
        self.confirmation = None
        self.status_msg = status_msg

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.pub_status = self.node.create_publisher(String, '/mission/robot_status', 10)
        self.sub_conf = self.node.create_subscription(Bool, '/mission/confirmation', self._cb, 10)

    def initialise(self):
        self.confirmation = None
        msg = String()
        msg.data = self.status_msg
        self.pub_status.publish(msg)
        self.node.get_logger().info(f"[{self.name}] Waiting for confirmation ({self.status_msg})...")

    def _cb(self, msg):
        self.confirmation = msg.data

    def update(self):
        if self.confirmation is None:
            return py_trees.common.Status.RUNNING

        if self.confirmation:
            self.node.get_logger().info(f"[{self.name}] Confirmed (YES).")
            return py_trees.common.Status.SUCCESS
        else:
            self.node.get_logger().info(f"[{self.name}] Rejected (NO).")
            return py_trees.common.Status.FAILURE


class CatchObject(py_trees.behaviour.Behaviour):
    """
    Activates the gripper to catch an object.
    """

    def __init__(self, name: str = "Catch Gripper"):
        super(CatchObject, self).__init__(name)
        self.node: Optional[Node] = None
        self.pub_catch = None
        self.start_time = None
        self.duration = 4.0

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.pub_catch = self.node.create_publisher(Bool, '/catch', 10)

    def initialise(self):
        self.node.get_logger().info("[Action] Closing Gripper.")
        msg = Bool()
        msg.data = True
        self.pub_catch.publish(msg)
        self.start_time = self.node.get_clock().now()

    def update(self):
        now = self.node.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9
        if elapsed > self.duration:
            self.node.get_logger().info("[Action] Object caught.")
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class ToggleExploration(py_trees.behaviour.Behaviour):
    """
    Enables or disables the exploration node (explore_lite).
    """

    def __init__(self, name: str = "Toggle Explore", enable: bool = True, min_publish_time: float = 0.5):
        super().__init__(name)
        self.enable = enable
        self.min_publish_time = min_publish_time
        self.node: Optional[Node] = None
        self.pub = None
        self.start_time = None
        self.has_logged = False

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.pub = self.node.create_publisher(Bool, 'explore/resume', 10)

    def initialise(self):
        self.start_time = self.node.get_clock().now()
        self.has_logged = False

    def update(self):
        msg = Bool()
        msg.data = self.enable
        self.pub.publish(msg)

        if not self.has_logged:
            state = "ON" if self.enable else "OFF"
            self.node.get_logger().info(f"[Action] Exploration {state} (publishing...)")
            self.has_logged = True

        if self.pub.get_subscription_count() == 0:
            return py_trees.common.Status.RUNNING

        elapsed = (self.node.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed < self.min_publish_time:
            return py_trees.common.Status.RUNNING

        return py_trees.common.Status.SUCCESS


class ForceFailure(py_trees.behaviour.Behaviour):
    """
    Always returns FAILURE. Used to control the flow in Selector/Retry loops.
    """

    def __init__(self, name: str = "Force Fail"):
        super(ForceFailure, self).__init__(name)

    def update(self):
        return py_trees.common.Status.FAILURE


class OpenGripper(py_trees.behaviour.Behaviour):
    """
    Opens the gripper by sending False to the /catch topic.
    """

    def __init__(self, name: str = "Open Gripper"):
        super(OpenGripper, self).__init__(name)
        self.node: Optional[Node] = None
        self.pub_catch = None
        self.start_time = None
        self.duration = 2.0

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.pub_catch = self.node.create_publisher(Bool, '/catch', 10)

    def initialise(self):
        self.node.get_logger().info("[Action] Opening Gripper.")
        msg = Bool()
        msg.data = False
        self.pub_catch.publish(msg)
        self.start_time = self.node.get_clock().now()

    def update(self):
        now = self.node.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9
        if elapsed > self.duration:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING