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
    RotateToTarget,
    WaitForStartSignal,
    WaitDuration,
    WaitForConfirmation,
    WaitForSkipSignal,
    BackUp,
    ForceFailure, PublishStatus,
    WaitForAbortSignal,
    OpenGripper, ManualRecovery
)


def create_tree(node: Node):
    """
    Architecture Finale : Exploration OneShot + Nav Skipable + Alignement + Retry Loop + Abort + Home Skipable
    """

    # --- HELPER : Création de la séquence Retour Base ---
    # On utilise une fonction pour pouvoir mettre le retour base à plusieurs endroits
    def create_home_sequence(suffix_name):
        home_seq = py_trees.composites.Parallel(
            name=f"Retour ou Skip {suffix_name}",
            policy=py_trees.common.ParallelPolicy.SuccessOnOne()
        )
        go_home = GoToHome(name=f"Retour Base {suffix_name}", node=node)
        wait_skip = WaitForSkipSignal(name=f"Skip Retour {suffix_name}")
        home_seq.add_children([go_home, wait_skip])
        return home_seq

    # --- RACINE ---
    root = py_trees.composites.Sequence(name="Mission_Supervisor", memory=True)

    # =========================================================================
    # --- PHASE 0 : SÉCURITÉ & ATTENTE ---
    # =========================================================================
    phase_init_sequence = py_trees.composites.Sequence(name="Phase 0: Seq", memory=True)
    safety_stop = ToggleExploration(name="Safety Stop (Init)", enable=False)
    init_open = OpenGripper(name="Init: Ouvrir Pince")
    wait_start = WaitForStartSignal(name="Attente GO")
    phase_init_sequence.add_children([safety_stop, init_open, wait_start])

    phase_init_oneshot = py_trees.decorators.OneShot(
        child=phase_init_sequence,
        name="Init Unique",
        policy=py_trees.common.OneShotPolicy.ON_SUCCESSFUL_COMPLETION
    )

    # =========================================================================
    # --- PHASE 1 : EXPLORATION ---
    # =========================================================================
    phase_explore_sequence = py_trees.composites.Sequence(name="Phase 1: Seq", memory=True)
    action_explore_on = ToggleExploration(name="Auto Explore ON", enable=True)

    scan_and_wait = py_trees.composites.Parallel(
        name="Scanning...",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne()
    )
    recorder = ObjectRecorder(name="Scanner 30cm")
    timer_explore = WaitDuration(name="Timer 100", duration=100.0)
    scan_and_wait.add_children([recorder, timer_explore])

    phase_explore_sequence.add_children([action_explore_on, scan_and_wait])

    phase_explore_oneshot = py_trees.decorators.OneShot(
        child=phase_explore_sequence,
        name="Exploration Unique",
        policy=py_trees.common.OneShotPolicy.ON_SUCCESSFUL_COMPLETION
    )

    # =========================================================================
    # --- PHASE 2 : SÉLECTION ---
    # =========================================================================
    phase_select = py_trees.composites.Sequence(name="Phase 2: Sélection", memory=True)
    stop_explore = ToggleExploration(name="Stop Explore", enable=False)
    user_choice = WaitForUserSelection(name="Menu Console")
    phase_select.add_children([stop_explore, user_choice])

    # =========================================================================
    # --- PHASE 3 : RÉCUPÉRATION (FETCH) ---
    # =========================================================================
    phase_fetch = py_trees.composites.Sequence(name="Phase 3: Fetch", memory=True)

    # 1. NAVIGATION NAV2
    move_sequence = py_trees.composites.Sequence(name="Séquence Déplacement", memory=True)
    nav_approach = GoToDetectedTarget(name="Approche Rapide (Nav2)")
    move_sequence.add_children([nav_approach])

    approach_with_skip = py_trees.composites.Parallel(
        name="Approche ou Skip",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne()
    )
    wait_skip = WaitForSkipSignal(name="Bouton Skip")
    approach_with_skip.add_children([move_sequence, wait_skip])

    # 2. BOUCLE D'APPROCHE FINALE
    attempt_sequence = py_trees.composites.Sequence(name="Tentative Catch", memory=True)

    ask_alignment = WaitForConfirmation(name="Demande Alignement", status_msg="WAITING_ALIGNMENT")
    rotate_to_target = RotateToTarget(name="Rotation Visuelle", threshold=0.05)
    vis_approach = VisualServoingApproach(name="Avance Fine")

    decision_selector = py_trees.composites.Selector(name="Validation ou Manuel", memory=False)

    # --- BRANCHE AUTO ---
    commit_sequence = py_trees.composites.Sequence(name="Branche: AUTO", memory=True)
    ask_catch_confirm = WaitForConfirmation(name="Validation Catch", status_msg="WAITING_CATCH")
    action_catch = CatchObject(name="Action Catch")
    verify_catch = WaitForConfirmation(name="Vérification Prise", status_msg="WAITING_CATCH_VERIFICATION")

    # AJOUT : Le retour base est ICI (seulement si succès auto)
    home_auto = create_home_sequence("(Auto)")

    commit_sequence.add_children([ask_catch_confirm, action_catch, verify_catch, home_auto])

    # --- BRANCHE MANUEL ---
    manual_sequence = py_trees.composites.Sequence(name="Branche: MANUEL", memory=True)
    start_manual = ManualRecovery(name="Pilotage Manuel")

    # AJOUT : Le retour base est AUSSI ICI (seulement si succès manuel)
    home_manual = create_home_sequence("(Manuel)")

    manual_sequence.add_children([start_manual, home_manual])

    # Assemblage sélecteur
    decision_selector.add_children([commit_sequence, manual_sequence])

    # Assemblage tentative
    attempt_sequence.add_children([ask_alignment, rotate_to_target, vis_approach, decision_selector])

    # Enveloppe Retry
    final_approach_loop = py_trees.decorators.Retry(
        child=attempt_sequence,
        name="Boucle Approche/Retry",
        num_failures=100
    )

    # Enveloppe Abort
    loop_or_abort = py_trees.composites.Parallel(
        name="Boucle ou Abort",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne()
    )
    wait_abort = WaitForAbortSignal(name="Bouton Abort")
    loop_or_abort.add_children([final_approach_loop, wait_abort])

    # 3. SIGNAL FIN (IDLE)
    # Note : On a supprimé le "home_with_skip" global qui était ici.
    # Si on sort par "wait_abort" (votre cas "Non"), on passe direct à IDLE.
    signal_idle = PublishStatus(name="Signal Fin Mission", status="IDLE")

    # =========================================================================
    # --- ASSEMBLAGE FINAL ---
    # =========================================================================
    phase_fetch.add_children([approach_with_skip, loop_or_abort, signal_idle])

    root.add_children([phase_init_oneshot, phase_explore_oneshot, phase_select, phase_fetch])

    return root


def main():
    rclpy.init()
    node = Node("bt_supervisor")

    root = create_tree(node)
    tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=False)

    try:
        tree.setup(node=node, timeout=15.0)
        print("Superviseur Prêt. Lancez le Mission Controller pour démarrer.")
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