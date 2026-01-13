#!/usr/bin/env python3
import rclpy
import py_trees
import py_trees_ros.trees
from rclpy.node import Node
import py_trees.visitors

from tb3_autonomy.behaviors.vision import ObjectRecorder
from tb3_autonomy.behaviors.navigation import GoToDetectedTarget, GoToHome
from tb3_autonomy.behaviors.actions import (
    VisualServoingApproach,
    CatchObject,
    ToggleExploration,
    WaitForUserSelection,
    WaitForStartSignal, WaitDuration, WaitForConfirmation, WaitForSkipSignal
)


def create_tree(node: Node):
    """
    Architecture corrigée avec OneShot pour éviter la ré-exploration
    """

    # --- RACINE ---
    root = py_trees.composites.Sequence(name="Mission_Supervisor", memory=True)

    # --- PHASE 0 : INIT ---
    phase_init = py_trees.composites.Sequence(name="Phase 0: Init", memory=False)
    safety_stop = ToggleExploration(name="Safety Stop (Init)", enable=False)
    wait_start = WaitForStartSignal(name="Attente GO")
    phase_init.add_children([safety_stop, wait_start])

    # --- PHASE 1 : EXPLORATION ---
    phase_explore_sequence = py_trees.composites.Sequence(name="Phase 1: Seq", memory=True)

    action_explore_on = ToggleExploration(name="Auto Explore ON", enable=True)

    scan_and_wait = py_trees.composites.Parallel(
        name="Scanning...",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne()
    )
    recorder = ObjectRecorder(name="Scanner 30cm")
    timer_explore = WaitDuration(name="Timer 60s", duration=60.0)

    scan_and_wait.add_children([recorder, timer_explore])
    phase_explore_sequence.add_children([action_explore_on, scan_and_wait])

    # =========================================================================
    # CORRECTION 1 : ON ENVELOPPE LA PHASE 1 DANS UN ONESHOT
    # =========================================================================
    phase_explore_oneshot = py_trees.decorators.OneShot(
        child=phase_explore_sequence,
        name="Exploration Unique",
        policy=py_trees.common.OneShotPolicy.ON_SUCCESSFUL_COMPLETION  # <--- LIGNE AJOUTÉE
    )

    # --- PHASE 2 : SÉLECTION ---
    phase_select = py_trees.composites.Sequence(name="Phase 2: Sélection", memory=True)
    stop_explore = ToggleExploration(name="Stop Explore", enable=False)
    user_choice = WaitForUserSelection(name="Menu Console")
    phase_select.add_children([stop_explore, user_choice])

    # --- PHASE 3 : FETCH ---
    phase_fetch = py_trees.composites.Sequence(name="Phase 3: Fetch", memory=True)

    move_sequence = py_trees.composites.Sequence(name="Séquence Déplacement", memory=True)
    nav_approach = GoToDetectedTarget(name="Approche Rapide (Nav2)")
    vis_approach = VisualServoingApproach(name="Approche Fine (Visual)")
    move_sequence.add_children([nav_approach, vis_approach])

    approach_with_skip = py_trees.composites.Parallel(
        name="Approche ou Skip",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne()
    )
    wait_skip = WaitForSkipSignal(name="Bouton Skip")
    approach_with_skip.add_children([move_sequence, wait_skip])

    ask_confirm = WaitForConfirmation(name="Validation Humaine")
    action_catch = CatchObject(name="Action Catch")
    go_home = GoToHome(name="Retour Base", node=node)

    phase_fetch.add_children([approach_with_skip, ask_confirm, action_catch, go_home])

    root.add_children([phase_init, phase_explore_oneshot, phase_select, phase_fetch])

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
        print("Superviseur Prêt. Lancez le Mission Controller pour démarrer.")

        # On garde le TickRate à 100ms (10Hz)
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