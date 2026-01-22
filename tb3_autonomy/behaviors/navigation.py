#!/usr/bin/env python3
import rclpy
import py_trees
from typing import Optional, Any

from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

# Constants for Home Position
HOME_POS_X = -2.0
HOME_POS_Y = -0.5
HOME_POS_Z = 0.0
HOME_ORIENT_W = 1.0


class GoToDetectedTarget(py_trees.behaviour.Behaviour):
    """
    Sends the robot to a target pose stored in the Blackboard.

    This behavior retrieves the 'target_pose_map' from the blackboard
    and sends it to the Nav2 'navigate_to_pose' action server.
    """

    def __init__(self, name: str = "GoTo Target"):
        super(GoToDetectedTarget, self).__init__(name)
        self.node: Optional[Node] = None
        self.action_client: Optional[ActionClient] = None
        self.goal_handle = None
        self.get_result_future = None
        self.server_online = False
        self.send_goal_future = None

        self.blackboard = py_trees.blackboard.Client(name="Nav")
        self.blackboard.register_key(key="target_pose_map", access=py_trees.common.Access.READ)

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.action_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')

    def initialise(self):
        self.goal_handle = None
        self.get_result_future = None
        self.server_online = False

        target_pose = self.blackboard.target_pose_map
        if target_pose is None:
            self.node.get_logger().error(f"[{self.name}] No target pose in blackboard.")
            return

        # Wait for server with a timeout (5s to account for potential lag)
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error(f"[{self.name}] Nav2 server unavailable (Timeout).")
            return

        self.server_online = True

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = target_pose

        self.node.get_logger().info(f"[{self.name}] Sending Nav2 goal to X={target_pose.pose.position.x:.2f}")

        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Callback triggered when the action server accepts or rejects the goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().warn(f"[{self.name}] Goal rejected by Nav2.")
            return

        self.goal_handle = goal_handle
        self.get_result_future = goal_handle.get_result_async()

    def update(self):
        # 1. Check if server was online at initialization
        if not self.server_online:
            return py_trees.common.Status.FAILURE

        # 2. Check if goal submission failed (rejected or timeout)
        if self.goal_handle is None:
            if self.send_goal_future and self.send_goal_future.done():
                return py_trees.common.Status.FAILURE
            return py_trees.common.Status.RUNNING

        # 3. Check for final result
        if self.get_result_future is None:
            return py_trees.common.Status.RUNNING

        if self.get_result_future.done():
            status = self.get_result_future.result().status
            # Status 4 = SUCCEEDED
            if status == 4:
                self.node.get_logger().info(f"[{self.name}] Target reached.")
                return py_trees.common.Status.SUCCESS
            else:
                self.node.get_logger().warn(f"[{self.name}] Navigation failed or canceled (Code {status}).")
                return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID:
            self._cancel_goal()

    def _cancel_goal(self):
        if self.goal_handle is not None and self.get_result_future is not None and not self.get_result_future.done():
            try:
                self.goal_handle.cancel_goal_async()
            except Exception:
                pass
        self.goal_handle = None


class GoToHome(py_trees.behaviour.Behaviour):
    """
    Sends the robot back to the base station (Home).

    Fault Tolerance:
    This behavior is designed to return SUCCESS even if navigation fails or
    is rejected. This ensures the behavior tree can proceed to the 'IDLE'
    state and restore control to the user, rather than getting stuck.
    """

    def __init__(self, name: str = "Return Home", node: Optional[Node] = None):
        super(GoToHome, self).__init__(name)
        self.node = node
        self.action_client: Optional[ActionClient] = None
        self.goal_handle = None
        self.get_result_future = None
        self.server_online = False
        self.send_goal_future = None

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.action_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')

    def initialise(self):
        self.node.get_logger().info(f"[{self.name}] Returning to base ({HOME_POS_X}, {HOME_POS_Y})...")
        self.goal_handle = None
        self.get_result_future = None
        self.server_online = False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"

        # Set Home Position
        goal_msg.pose.pose.position.x = HOME_POS_X
        goal_msg.pose.pose.position.y = HOME_POS_Y
        goal_msg.pose.pose.position.z = HOME_POS_Z

        # Set Neutral Orientation (Facing East)
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = HOME_ORIENT_W

        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error(f"[{self.name}] Nav2 server unavailable (Timeout).")
            return

        self.server_online = True

        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().warn(f"[{self.name}] Home goal rejected by Nav2.")
            return

        self.goal_handle = goal_handle
        self.get_result_future = goal_handle.get_result_async()

    def update(self):
        # 1. Fault Tolerance: Force SUCCESS if server is down
        if not self.server_online:
            self.node.get_logger().warn(f"[{self.name}] Server offline. Forcing SUCCESS to release control.")
            return py_trees.common.Status.SUCCESS

        # 2. Fault Tolerance: Force SUCCESS if goal rejected
        if self.goal_handle is None:
            if self.send_goal_future and self.send_goal_future.done():
                self.node.get_logger().warn(f"[{self.name}] Goal rejected. Forcing SUCCESS to release control.")
                return py_trees.common.Status.SUCCESS
            return py_trees.common.Status.RUNNING

        # 3. Wait for result
        if self.get_result_future is None:
            return py_trees.common.Status.RUNNING

        if self.get_result_future.done():
            self.node.get_logger().info(f"[{self.name}] Return to base sequence finished.")
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID:
            if self.goal_handle:
                self.goal_handle.cancel_goal_async()
                self.goal_handle = None