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
    WaitForStartSignal
)


def create_tree(node: Node):
    """
    Architecture corrigée pour éviter le skip de l'exploration
    """

    # --- PHASE 0 : SÉCURITÉ & ATTENTE ---
    # CORRECTION CRITIQUE 1 : memory=False ici !
    # Cela force l'arbre à ré-exécuter 'safety_stop' à CHAQUE tick tant qu'on attend.
    # On spamme donc l'ordre "STOP" pour être sûr que explore_lite ne démarre pas.
    root = py_trees.composites.Sequence(name="Mission_Supervisor", memory=True)

    # --- CORRECTION 1 : memory=True pour arrêter le spam ---
    # Une fois que Safety Stop a réussi, on ne le refait plus, on attend juste le GO.
    phase_init = py_trees.composites.Sequence(name="Phase 0: Init", memory=False)

    safety_stop = ToggleExploration(name="Safety Stop (Init)", enable=False)
    wait_start = WaitForStartSignal(name="Attente GO")

    phase_init.add_children([safety_stop, wait_start])

    # --- PHASE 1 : EXPLORATION & SCAN ---
    # CORRECTION CRITIQUE 2 : On sépare l'activation du Timer.
    # Si on met tout dans un Parallel(SuccessOnOne), l'activation (qui est instantanée)
    # fait terminer la phase tout de suite.
    phase_explore_sequence = py_trees.composites.Sequence(name="Phase 1: Seq", memory=True)

    # Étape 1 : On allume
    action_explore_on = ToggleExploration(name="Auto Explore ON", enable=True)

    # Étape 2 : On attend (Timer) tout en scannant (Recorder)
    # C'est ce bloc qui va durer 60s
    scan_and_wait = py_trees.composites.Parallel(
        name="Scanning...",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne()
    )

    recorder = ObjectRecorder(name="Scanner 30cm")

    timer_explore = py_trees.decorators.Timeout(
        name="Timer 60s",
        child=py_trees.behaviours.Running(name="Attente..."),
        duration=60.0
    )

    scan_and_wait.add_children([recorder, timer_explore])

    # On ajoute les deux étapes à la séquence de la Phase 1
    phase_explore_sequence.add_children([action_explore_on, scan_and_wait])

    # --- PHASE 2 : CHOIX UTILISATEUR ---
    phase_select = py_trees.composites.Sequence(name="Phase 2: Sélection", memory=True)

    stop_explore = ToggleExploration(name="Stop Explore", enable=False)
    # Note : node=node est nécessaire ici car on utilise 'self.node' dans le __init__ de votre action
    user_choice = WaitForUserSelection(name="Menu Console")

    phase_select.add_children([stop_explore, user_choice])

    # --- PHASE 3 : RÉCUPÉRATION ---
    phase_fetch = py_trees.composites.Sequence(name="Phase 3: Fetch", memory=True)

    nav_approach = GoToDetectedTarget(name="Approche Rapide (Nav2)")
    vis_approach = VisualServoingApproach(name="Approche Fine (Visual)")
    action_catch = CatchObject(name="Action Catch")
    # Note : node=node nécessaire ici aussi pour GoToHome
    go_home = GoToHome(name="Retour Base", node=node)

    phase_fetch.add_children([nav_approach, vis_approach, action_catch, go_home])

    # --- ASSEMBLAGE ---
    root.add_children([phase_init, phase_explore_sequence, phase_select, phase_fetch])
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
        tree.tick_tock(period_ms=100)
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()