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
    ForceFailure,
    WaitForAbortSignal,
    OpenGripper  # <--- AJOUT IMPORTANT
)


def create_tree(node: Node):
    """
    Architecture Finale : Exploration OneShot + Nav Skipable + Alignement + Retry Loop + Abort + Home Skipable
    """

    # --- RACINE ---
    root = py_trees.composites.Sequence(name="Mission_Supervisor", memory=True)

    # =========================================================================
    # --- PHASE 0 : SÉCURITÉ & ATTENTE ---
    # =========================================================================
    # CORRECTION : memory=True pour éviter la boucle infinie "Ouverture Pince"
    phase_init_sequence = py_trees.composites.Sequence(name="Phase 0: Seq", memory=True)

    safety_stop = ToggleExploration(name="Safety Stop (Init)", enable=False)

    init_open = OpenGripper(name="Init: Ouvrir Pince")

    wait_start = WaitForStartSignal(name="Attente GO")

    phase_init_sequence.add_children([safety_stop, wait_start,init_open])

    # Protection OneShot pour ne pas refaire l'init si l'arbre redémarre
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
    timer_explore = WaitDuration(name="Timer 60s", duration=60.0)

    scan_and_wait.add_children([recorder, timer_explore])

    phase_explore_sequence.add_children([action_explore_on, scan_and_wait])

    # Protection OneShot pour ne pas relancer l'exploration après une mission
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

    # ---------------------------------------------------------
    # 1. NAVIGATION NAV2 (Groupe Nav + Skip)
    # ---------------------------------------------------------
    move_sequence = py_trees.composites.Sequence(name="Séquence Déplacement", memory=True)
    nav_approach = GoToDetectedTarget(name="Approche Rapide (Nav2)")
    move_sequence.add_children([nav_approach])

    approach_with_skip = py_trees.composites.Parallel(
        name="Approche ou Skip",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne()
    )
    wait_skip = WaitForSkipSignal(name="Bouton Skip")
    approach_with_skip.add_children([move_sequence, wait_skip])

    # ---------------------------------------------------------
    # 2. BOUCLE D'APPROCHE FINALE (Alignement -> Avance -> Catch)
    # ---------------------------------------------------------
    attempt_sequence = py_trees.composites.Sequence(name="Tentative Catch", memory=True)

    # Etape A : Demande si on peut s'aligner (WAITING_ALIGNMENT)
    ask_alignment = WaitForConfirmation(
        name="Demande Alignement",
        status_msg="WAITING_ALIGNMENT"
    )

    # Etape B : Rotation pure
    rotate_to_target = RotateToTarget(name="Rotation Visuelle", threshold=0.05)

    # Etape C : Avance pure
    vis_approach = VisualServoingApproach(name="Avance Fine")

    # Etape D : Décision Finale (Catch ou Retry)
    decision_selector = py_trees.composites.Selector(name="Validation ou Recul", memory=False)

    # D.1 : Branche OUI (Catch + Vérif)
    # ---------------------------------
    commit_sequence = py_trees.composites.Sequence(name="Branche: OUI", memory=True)

    # 1. On demande confirmation pour pincer
    ask_catch_confirm = WaitForConfirmation(name="Validation Catch", status_msg="WAITING_CATCH")
    # 2. On pince
    action_catch = CatchObject(name="Action Catch")
    # 3. On vérifie si c'est bon (WAITING_CATCH_VERIFICATION)
    verify_catch = WaitForConfirmation(name="Vérification Prise", status_msg="WAITING_CATCH_VERIFICATION")

    # CORRECTION : On ajoute les 3 étapes proprement
    commit_sequence.add_children([ask_catch_confirm, action_catch, verify_catch])


    # D.2 : Branche NON (Recul + Ouverture)
    # -------------------------------------
    retry_sequence = py_trees.composites.Sequence(name="Branche: NON", memory=True)

    back_up = BackUp(name="Reculer", duration=2.5, speed=-0.15)

    # AJOUT : On ré-ouvre la pince si on a raté
    re_open = OpenGripper(name="Retry: Ouvrir Pince")

    force_fail = ForceFailure(name="Restart Loop")

    retry_sequence.add_children([back_up, re_open, force_fail])


    # -- Assemblage du sélecteur --
    decision_selector.add_children([commit_sequence, retry_sequence])

    # -- Assemblage de la séquence de tentative --
    attempt_sequence.add_children([ask_alignment, rotate_to_target, vis_approach, decision_selector])

    # -- Enveloppe RETRY (Réessaie indéfiniment tant que pas Catch) --
    final_approach_loop = py_trees.decorators.Retry(
        child=attempt_sequence,
        name="Boucle Approche/Retry",
        num_failures=100
    )

    # -- Enveloppe ABORT (Permet de quitter la boucle Retry) --
    loop_or_abort = py_trees.composites.Parallel(
        name="Boucle ou Abort",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne()
    )
    wait_abort = WaitForAbortSignal(name="Bouton Abort")
    loop_or_abort.add_children([final_approach_loop, wait_abort])

    # ---------------------------------------------------------
    # 3. RETOUR BASE (Skipable)
    # ---------------------------------------------------------
    # Note : Le robot rentrera toujours après un succès ici.
    home_with_skip = py_trees.composites.Parallel(
        name="Retour ou Skip",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne()
    )

    go_home = GoToHome(name="Retour Base", node=node)
    wait_skip_home = WaitForSkipSignal(name="Skip Retour") # Instance séparée !

    home_with_skip.add_children([go_home, wait_skip_home])


    # =========================================================================
    # --- ASSEMBLAGE FINAL DE LA PHASE 3 ---
    # =========================================================================
    phase_fetch.add_children([approach_with_skip, loop_or_abort, home_with_skip])

    # =========================================================================
    # --- RACINE ---
    # =========================================================================
    root.add_children([phase_init_oneshot, phase_explore_oneshot, phase_select, phase_fetch])

    return root


def main():
    rclpy.init()
    node = Node("bt_supervisor")

    # Création de l'arbre
    root = create_tree(node)

    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=False
    )

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