#!/usr/bin/env python3
import sys
import time
import threading
import select
import termios
import tty
from typing import List, Tuple, Dict

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool, String
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Twist

# Constants
STEP_DURATION = 0.2
MOVE_BINDINGS: Dict[str, Tuple[float, float]] = {
    'z': (0.15, 0.0),  # Forward
    's': (-0.15, 0.0),  # Backward
    'q': (0.0, 0.5),  # Left
    'd': (0.0, -0.5),  # Right
    'x': (0.0, 0.0),  # Stop
}


class MissionController(Node):
    """
    User Interface Node for the TurtleBot3 Autonomy Mission.

    Handles:
    - User input for target selection.
    - Teleoperation in manual recovery mode.
    - Status display and workflow confirmation.
    """

    def __init__(self):
        super().__init__('mission_controller')

        # Publishers
        self.pub_choice = self.create_publisher(Int32, '/mission/select_target', 10)
        self.pub_start = self.create_publisher(Bool, '/mission/start', 10)
        self.pub_confirm = self.create_publisher(Bool, '/mission/confirmation', 10)
        self.pub_abort = self.create_publisher(Bool, '/mission/abort', 10)
        self.pub_skip = self.create_publisher(Bool, '/mission/skip_nav', 10)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_catch = self.create_publisher(Bool, '/catch', 10)

        # Subscribers
        self.sub_markers = self.create_subscription(MarkerArray, '/supervisor/known_objects', self.markers_cb, 10)
        self.sub_status = self.create_subscription(String, '/mission/robot_status', self.status_cb, 10)

        # State & Settings
        self.settings = termios.tcgetattr(sys.stdin)
        self.known_ids: List[int] = []
        self.robot_state: str = "IDLE"
        self.gripper_closed: bool = False

    def markers_cb(self, msg: MarkerArray):
        """
        Callback to update the list of available target IDs from visualization markers.
        """
        current_ids = []
        for m in msg.markers:
            if m.ns == "ids":
                try:
                    # Extracts ID from marker text (e.g., "ID 2" -> 2)
                    txt = m.text.split()[1]
                    obj_id = int(txt)
                    current_ids.append(obj_id)
                except (IndexError, ValueError):
                    pass
        current_ids.sort()
        self.known_ids = current_ids

    def status_cb(self, msg: String):
        """
        Updates the internal state machine based on the supervisor's status.
        """
        status_map = {
            "WAITING_ALIGNMENT": "READY_TO_ALIGN",
            "WAITING_CATCH": "READY_TO_CATCH",
            "WAITING_CONFIRMATION": "READY_TO_CATCH",
            "WAITING_CATCH_VERIFICATION": "READY_TO_VERIFY",
            "MANUAL_RECOVERY": "MANUAL_MODE",
            "IDLE": "IDLE"
        }

        if msg.data in status_map:
            self.robot_state = status_map[msg.data]

    def get_key(self) -> str:
        """
        Captures a single key press from stdin without blocking.
        """
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run_teleop_session(self):
        """
        Starts the manual control loop (Teleop) for recovery.
        """
        print("\n" + "-" * 50)
        print("MANUAL MODE: STEP-BY-STEP CONTROL")
        print("-" * 50)
        print(" [z]: Forward")
        print(" [s]: Backward")
        print(" [q]: Left")
        print(" [d]: Right")
        print(" [SPACE]: Toggle Gripper")
        print(" [ENTER]: Finish & Verify")
        print("-" * 50)

        self.gripper_closed = False

        while rclpy.ok():
            key = self.get_key()

            # --- Step-by-Step Movement ---
            if key in MOVE_BINDINGS:
                v, w = MOVE_BINDINGS[key]

                # 1. Send velocity command
                twist = Twist()
                twist.linear.x = float(v)
                twist.angular.z = float(w)
                self.pub_cmd_vel.publish(twist)

                # 2. Wait for step duration (unless stopping)
                if key != 'x':
                    time.sleep(STEP_DURATION)

                # 3. Stop immediately after step
                self.pub_cmd_vel.publish(Twist())

            # --- Gripper Control ---
            elif key == ' ':
                self.gripper_closed = not self.gripper_closed
                state = "CLOSED" if self.gripper_closed else "OPEN"
                print(f"\r Gripper: {state:<10}", end="", flush=True)
                self.pub_catch.publish(Bool(data=self.gripper_closed))

            # --- Exit Teleop ---
            elif key in ['\r', '\n']:
                # Ensure stop
                self.pub_cmd_vel.publish(Twist())
                print("\n\n>>> Ending manual control.")

                while True:
                    resp = input("[Query] Did you catch the object? (y/n): ").lower()
                    if resp in ['o', 'y', 'oui', 'yes']:
                        print(">> SUCCESS CONFIRMED. Returning to base.")
                        # Send confirmation multiple times to ensure reception
                        for _ in range(10):
                            self.pub_confirm.publish(Bool(data=True))
                            time.sleep(0.05)
                        self.robot_state = "MOVING"
                        return
                    elif resp in ['n', 'non', 'no']:
                        print(">> FAILED. Resuming manual control...")
                        self.gripper_closed = False
                        self.pub_catch.publish(Bool(data=False))
                        print(">> Gripper reset. Ready.")
                        break
                    else:
                        print("Invalid input. Please type 'y' or 'n'.")

            elif key == '\x03':  # Ctrl+C
                break

    def run_interface(self):
        """
        Main loop handling the CLI menu and high-level workflow.
        """
        # --- PHASE 1 : START ---
        print("\n" + "=" * 40)
        print("MISSION CONTROL READY")
        print("=" * 40)
        input(">>> Press [ENTER] to start exploration...")

        self.pub_start.publish(Bool(data=True))
        print(">> Signal sent. Exploration started...")

        # --- MAIN LOOP ---
        while rclpy.ok():
            # Only show menu if robot is IDLE
            if self.robot_state == "IDLE":
                print("\n" + "-" * 40)
                print("SELECTION MENU (Press ENTER to refresh)")
                print("-" * 40)
                input()  # Wait for user to press Enter

                if not self.known_ids:
                    print(f"\rWaiting for objects... (Found: {len(self.known_ids)})", end="")
                    time.sleep(1)
                    continue

                try:
                    print(f"AVAILABLE OBJECTS: {self.known_ids}")
                    raw = input("Enter Target ID (or 'wait'): ")

                    if self.robot_state != "IDLE":
                        continue

                    if raw == 'wait':
                        continue

                    choice = int(raw)
                    if choice in self.known_ids:
                        print(f" >> Target #{choice} selected. Dispatching robot...")
                        self.pub_start.publish(Bool(data=True))
                        self.pub_choice.publish(Int32(data=choice))
                        self.robot_state = "MOVING"
                        self.wait_for_arrival()
                    else:
                        print("Unknown ID.")
                except ValueError:
                    print("Invalid input.")
                except Exception:
                    pass

            time.sleep(0.1)

    def wait_for_arrival(self):
        """
        Blocking loop that monitors the robot's progress during a mission
        and handles user interaction (Skip, Confirmation, Recovery).
        """
        print("Moving to target... [Press 'S' + Enter to Skip Navigation]")

        while rclpy.ok():
            # 1. SKIP COMMAND HANDLING
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                line = sys.stdin.readline().strip()
                if line.lower() == 's':
                    print("\n>> SKIP COMMAND SENT.")
                    self.pub_skip.publish(Bool(data=True))

            # 2. CASE A: Robot Arrived, Request Alignment
            if self.robot_state == "READY_TO_ALIGN":
                print("\n[Status] Robot arrived at destination.")

                while True:
                    q1 = input("[Query] Attempt to catch this object? (y/n): ").lower()
                    if q1 in ['n', 'non', 'no']:
                        print(">> ABORTING. Returning to menu.")
                        self.pub_abort.publish(Bool(data=True))
                        self.pub_confirm.publish(Bool(data=False))
                        self.robot_state = "IDLE"
                        return
                    elif q1 in ['o', 'y', 'oui', 'yes']:
                        print(">> Confirmed. Starting alignment and visual approach...")
                        self.pub_confirm.publish(Bool(data=True))
                        self.robot_state = "MOVING"
                        break
                    else:
                        print("Please answer 'y' or 'n'.")

            # 3. CASE B: Request Catch (Auto Mode)
            if self.robot_state == "READY_TO_CATCH":
                print("\n[Status] Robot aligned and close (approx 18cm).")

                while True:
                    resp = input("[Query] Execute catch sequence now? (y/n): ").lower()
                    if resp in ['o', 'y', 'oui', 'yes']:
                        self.pub_confirm.publish(Bool(data=True))
                        print(">> Catch command sent. Waiting for verification...")
                        self.robot_state = "MOVING"
                        break
                    elif resp in ['n', 'non', 'no']:
                        self.pub_confirm.publish(Bool(data=False))
                        print(">> User refused. Switching to manual recovery...")
                        self.robot_state = "MOVING"
                        break
                    else:
                        print("Please answer 'y' or 'n'.")

            # 4. CASE C: Verification (After Auto Catch)
            if self.robot_state == "READY_TO_VERIFY":
                print("\n[Query] Visual Check: Was the object caught?")

                while True:
                    verif = input("Success? (y/n): ").lower()
                    if verif in ['o', 'y', 'oui', 'yes']:
                        # Spam confirmation to ensure BT receives it
                        for _ in range(5):
                            self.pub_confirm.publish(Bool(data=True))
                            time.sleep(0.05)
                        print(">> SUCCESS CONFIRMED. Returning to base.")
                        self.robot_state = "MOVING"
                        break
                    elif verif in ['n', 'non', 'no']:
                        self.pub_confirm.publish(Bool(data=False))
                        print(">> FAILURE REPORTED. Switching to recovery/retry...")
                        self.robot_state = "MOVING"
                        break
                    else:
                        print("Please answer 'y' or 'n'.")

            # 5. CASE D: Manual Mode
            if self.robot_state == "MANUAL_MODE":
                self.run_teleop_session()
                # Reset local state to avoid re-triggering teleop loop immediately
                self.robot_state = "MOVING"

            # 6. End of Mission
            if self.robot_state == "IDLE":
                print("\n>>> Mission complete. Returning to main menu.")
                return

            time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)
    node = MissionController()

    # Spin in a separate thread to keep callbacks active while blocking on input
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.run_interface()
    except KeyboardInterrupt:
        print("\n[System] Controller shutting down.")
    except Exception as e:
        print(f"[Error] Unexpected error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()