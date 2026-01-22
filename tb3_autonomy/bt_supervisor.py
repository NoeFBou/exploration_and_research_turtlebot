#!/usr/bin/env python3
import rclpy
import py_trees
import py_trees_ros.trees
import py_trees.visitors
from rclpy.node import Node

from tb3_autonomy.behaviors.vision import ObjectRecorder
from tb3_autonomy.behaviors.navigation import GoToDetectedTarget, GoToHome
from tb3_autonomy.behaviors.actions import (
    VisualServoingApproach,
    CatchObject,
    ToggleExploration,
    WaitForUserSelection,
    RotateToTarget,
    WaitForStartSignal,
    WaitDuration,
    WaitForConfirmation,
    WaitForSkipSignal,
    PublishStatus,
    WaitForAbortSignal,
    OpenGripper,
    ManualRecovery
)


def create_tree(node: Node) -> py_trees.behaviour.Behaviour:
    """
    Constructs the Behavior Tree for the autonomous mission.

    Architecture Overview:
    1. Phase 0 (Init): Safety checks and waiting for start signal.
    2. Phase 1 (Explore): Autonomous exploration and object recording (OneShot).
    3. Phase 2 (Select): Waiting for user to select a target ID.
    4. Phase 3 (Fetch): Navigation, Visual Servoing, Catching (Auto/Manual), and Return.

    Args:
        node (Node): The ROS 2 node instance used by behaviors.

    Returns:
        py_trees.behaviour.Behaviour: The root of the behavior tree.
    """

    # --- HELPER: Return to Base Sequence Generator ---
    def create_home_sequence(suffix_name: str) -> py_trees.composites.Parallel:
        """Creates a parallel sequence for returning home that can be skipped."""
        home_seq = py_trees.composites.Parallel(
            name=f"Return Home or Skip {suffix_name}",
            policy=py_trees.common.ParallelPolicy.SuccessOnOne()
        )
        go_home = GoToHome(name=f"Go Home {suffix_name}", node=node)
        wait_skip = WaitForSkipSignal(name=f"Skip Return {suffix_name}")
        home_seq.add_children([go_home, wait_skip])
        return home_seq

    # --- ROOT ---
    root = py_trees.composites.Sequence(name="Mission Supervisor", memory=True)

    # =========================================================================
    # PHASE 0: INITIALIZATION & SAFETY
    # =========================================================================
    phase_init_sequence = py_trees.composites.Sequence(name="Phase 0: Init Sequence", memory=True)

    safety_stop = ToggleExploration(name="Safety Stop (Init)", enable=False)
    init_open = OpenGripper(name="Init: Open Gripper")
    wait_start = WaitForStartSignal(name="Wait for GO")

    phase_init_sequence.add_children([safety_stop, init_open, wait_start])

    # Wrap in OneShot to ensure it runs only once per session
    phase_init_oneshot = py_trees.decorators.OneShot(
        child=phase_init_sequence,
        name="Init OneShot",
        policy=py_trees.common.OneShotPolicy.ON_SUCCESSFUL_COMPLETION
    )

    # =========================================================================
    # PHASE 1: EXPLORATION
    # =========================================================================
    phase_explore_sequence = py_trees.composites.Sequence(name="Phase 1: Explore Sequence", memory=True)

    action_explore_on = ToggleExploration(name="Auto Explore ON", enable=True)

    # Run Recorder and Timer in parallel. Ends when Timer finishes.
    scan_and_wait = py_trees.composites.Parallel(
        name="Scanning...",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne()
    )
    recorder = ObjectRecorder(name="Object Recorder")
    timer_explore = WaitDuration(name="Exploration Timer", duration=100.0)

    scan_and_wait.add_children([recorder, timer_explore])

    phase_explore_sequence.add_children([action_explore_on, scan_and_wait])

    phase_explore_oneshot = py_trees.decorators.OneShot(
        child=phase_explore_sequence,
        name="Exploration OneShot",
        policy=py_trees.common.OneShotPolicy.ON_SUCCESSFUL_COMPLETION
    )

    # =========================================================================
    # PHASE 2: SELECTION
    # =========================================================================
    phase_select = py_trees.composites.Sequence(name="Phase 2: Selection", memory=True)

    stop_explore = ToggleExploration(name="Stop Explore", enable=False)
    user_choice = WaitForUserSelection(name="Wait User Selection")

    phase_select.add_children([stop_explore, user_choice])

    # =========================================================================
    # PHASE 3: FETCH & RETRIEVE
    # =========================================================================
    phase_fetch = py_trees.composites.Sequence(name="Phase 3: Fetch", memory=True)

    # --- 3.1: Navigation (Nav2) ---
    move_sequence = py_trees.composites.Sequence(name="Movement Sequence", memory=True)
    nav_approach = GoToDetectedTarget(name="Nav2 Approach")
    move_sequence.add_children([nav_approach])

    # Allow skipping the long-distance navigation
    approach_with_skip = py_trees.composites.Parallel(
        name="Approach or Skip",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne()
    )
    wait_skip = WaitForSkipSignal(name="Skip Navigation Button")
    approach_with_skip.add_children([move_sequence, wait_skip])

    # --- 3.2: Final Approach Loop (Visual Servoing -> Catch) ---
    attempt_sequence = py_trees.composites.Sequence(name="Catch Attempt Sequence", memory=True)

    ask_alignment = WaitForConfirmation(name="Request Alignment", status_msg="WAITING_ALIGNMENT")
    rotate_to_target = RotateToTarget(name="Visual Rotation", threshold=0.05)
    vis_approach = VisualServoingApproach(name="Fine Approach")

    # Selector: Try Auto Catch first, then Fallback to Manual
    decision_selector = py_trees.composites.Selector(name="Auto or Manual Selector", memory=False)

    # Branch A: Automatic Catch
    commit_sequence = py_trees.composites.Sequence(name="Branch: AUTO", memory=True)
    ask_catch_confirm = WaitForConfirmation(name="Confirm Catch", status_msg="WAITING_CATCH")
    action_catch = CatchObject(name="Close Gripper")
    verify_catch = WaitForConfirmation(name="Verify Grasp", status_msg="WAITING_CATCH_VERIFICATION")

    # Return home immediately after successful auto catch
    home_auto = create_home_sequence("(Auto)")

    commit_sequence.add_children([ask_catch_confirm, action_catch, verify_catch, home_auto])

    # Branch B: Manual Recovery
    manual_sequence = py_trees.composites.Sequence(name="Branch: MANUEL", memory=True)
    start_manual = ManualRecovery(name="Manual Control")

    # Return home immediately after successful manual catch
    home_manual = create_home_sequence("(Manual)")

    manual_sequence.add_children([start_manual, home_manual])

    decision_selector.add_children([commit_sequence, manual_sequence])

    attempt_sequence.add_children([ask_alignment, rotate_to_target, vis_approach, decision_selector])

    # Decorator: Retry the attempt sequence indefinitely until success
    final_approach_loop = py_trees.decorators.Retry(
        child=attempt_sequence,
        name="Retry Loop",
        num_failures=100
    )

    # Parallel: Allows aborting the retry loop
    loop_or_abort = py_trees.composites.Parallel(
        name="Loop or Abort",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne()
    )
    wait_abort = WaitForAbortSignal(name="Abort Button")
    loop_or_abort.add_children([final_approach_loop, wait_abort])

    # --- 3.3: Finish Signal ---
    # Resets the controller UI to IDLE state
    signal_idle = PublishStatus(name="Signal End Mission", status="IDLE")

    # Assemble Phase 3
    phase_fetch.add_children([approach_with_skip, loop_or_abort, signal_idle])

    # Assemble Root
    root.add_children([phase_init_oneshot, phase_explore_oneshot, phase_select, phase_fetch])

    return root


def main():
    rclpy.init()
    node = Node("bt_supervisor")

    root = create_tree(node)

    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=False
    )

    try:
        tree.setup(node=node, timeout=15.0)
        node.get_logger().info("Supervisor Ready. Launch Mission Controller to start.")

        # Attach snapshot visitor for debug visualization if needed
        tree.visitors.append(py_trees.visitors.SnapshotVisitor())

        tree.tick_tock(period_ms=100)
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()