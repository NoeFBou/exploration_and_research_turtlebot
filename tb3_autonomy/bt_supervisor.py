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

    #phase_init_sequence.add_children([safety_stop, wait_start,init_open])
    phase_init_sequence.add_children([safety_stop, init_open, wait_start])

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
    timer_explore = WaitDuration(name="Timer 100", duration=100.0)

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
    #decision_selector = py_trees.composites.Selector(name="Validation ou Recul", memory=False)

    # D.1 : Branche OUI (Catch + Vérif)
    # ---------------------------------
    decision_selector = py_trees.composites.Selector(name="Validation ou Manuel", memory=False)

    # D.1 : Branche OUI (Automatique)
    # ---------------------------------
    commit_sequence = py_trees.composites.Sequence(name="Branche: AUTO", memory=True)
    ask_catch_confirm = WaitForConfirmation(name="Validation Catch", status_msg="WAITING_CATCH")
    action_catch = CatchObject(name="Action Catch")
    verify_catch = WaitForConfirmation(name="Vérification Prise", status_msg="WAITING_CATCH_VERIFICATION")    # ATTENTION: Il faut retirer 'home_with_skip' d'ici pour le mettre APRÈS la boucle Retry globale
    # Si on le laisse ici, ça ne marchera pas pour le mode manuel.

    # Réorganisation pour supporter le manuel ET l'auto :
    # Si Auto réussit -> Success. Si Manuel réussit -> Success.
    # Si l'un des deux success, on sort du Selector, puis de la Retry Loop.

    commit_sequence.add_children([ask_catch_confirm, action_catch, verify_catch])


    # D.2 : Branche NON (Mode Manuel)
    # -------------------------------------
    # C'est ici qu'on change tout. Plus de recul simple, mais le mode manuel.
    manual_sequence = py_trees.composites.Sequence(name="Branche: MANUEL", memory=True)

    start_manual = ManualRecovery(name="Pilotage Manuel")

    # Si ManualRecovery renvoie SUCCESS (utilisateur a dit OUI),
    # alors manual_sequence renvoie SUCCESS.
    # Alors decision_selector renvoie SUCCESS.
    # Alors attempt_sequence renvoie SUCCESS.
    # Alors Retry Loop s'arrête (car c'est un Succès).
    # Et on pourra aller au Home.

    manual_sequence.add_children([start_manual])


    # -- Assemblage du sélecteur --
    decision_selector.add_children([commit_sequence, manual_sequence])

    # -- Assemblage de la séquence de tentative --
    attempt_sequence.add_children([ask_alignment, rotate_to_target, vis_approach, decision_selector])

    # -- Enveloppe RETRY --
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
    home_with_skip = py_trees.composites.Parallel(
        name="Retour ou Skip",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne()
    )
    go_home = GoToHome(name="Retour Base", node=node)
    wait_skip_home = WaitForSkipSignal(name="Skip Retour")
    home_with_skip.add_children([go_home, wait_skip_home])

    signal_idle = PublishStatus(name="Signal Fin Mission", status="IDLE")
    # =========================================================================
    # --- ASSEMBLAGE ---
    # =========================================================================
    phase_fetch.add_children([approach_with_skip, loop_or_abort, home_with_skip, signal_idle])

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