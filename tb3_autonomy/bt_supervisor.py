#!/usr/bin/env python3
import rclpy
import py_trees
import py_trees_ros.trees
import sys
from rclpy.node import Node
import py_trees.visitors
# Importation de vos comportements (Behaviors)
# Assurez-vous que les fichiers __init__.py existent dans le dossier behaviors/
from tb3_autonomy.behaviors.vision import ObjectRecorder
from tb3_autonomy.behaviors.navigation import GoToDetectedTarget, GoToHome
from tb3_autonomy.behaviors.actions import (
    VisualServoingApproach,
    CatchObject,
    ToggleExploration,
    WaitForUserSelection
)


def create_tree(node: Node):
    """
    Construit l'arbre de comportement.
    VERSION CORRIGÉE : Suppression des arguments 'node=node' inutiles dans les __init__
    """

    # --- RACINE ---
    root = py_trees.composites.Sequence(name="Mission_Supervisor", memory=True)

    # --- PHASE 1 : EXPLORATION & SCAN ---
    phase_explore = py_trees.composites.Parallel(
        name="Phase 1: Exploration",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne()
    )

    # CORRECTION ICI : On a retiré "node=node"
    action_explore = ToggleExploration(name="Auto Explore", enable=True)

    # CORRECTION ICI : On a retiré "node=node"
    recorder = ObjectRecorder(name="Scanner 30cm")

    # Timer
    timer_explore = py_trees.decorators.Timeout(
        name="Timer 60s",
        child=py_trees.behaviours.Running(name="Attente..."),  # <--- Changement ici
        duration=60.0
    )

    phase_explore.add_children([action_explore, recorder, timer_explore])

    # --- PHASE 2 : CHOIX UTILISATEUR ---
    phase_select = py_trees.composites.Sequence(name="Phase 2: Sélection", memory=True)

    # CORRECTION ICI : On a retiré "node=node"
    stop_explore = ToggleExploration(name="Stop Explore", enable=False)

    # ATTENTION : WaitForUserSelection a été défini avec 'node' dans le __init__ dans mon code précédent.
    # Si vous avez gardé mon code tel quel pour celui-ci, gardez node=node.
    # Sinon, retirez-le. Dans le doute, je laisse node=node pour celui-ci car c'est un ajout tardif.
    user_choice = WaitForUserSelection(name="Menu Console", node=node)

    phase_select.add_children([stop_explore, user_choice])

    # --- PHASE 3 : RÉCUPÉRATION ---
    phase_fetch = py_trees.composites.Sequence(name="Phase 3: Fetch", memory=True)

    # CORRECTION ICI : Suppression de node=node
    nav_approach = GoToDetectedTarget(name="Approche Rapide (Nav2)")

    # CORRECTION ICI : Suppression de node=node
    vis_approach = VisualServoingApproach(name="Approche Fine (Visual)")

    # CORRECTION ICI : Suppression de node=node
    action_catch = CatchObject(name="Action Catch")

    # ATTENTION : Idem que UserSelection, GoToHome avait 'node' dans son __init__
    go_home = GoToHome(name="Retour Base", node=node)

    phase_fetch.add_children([nav_approach, vis_approach, action_catch, go_home])

    # --- ASSEMBLAGE ---
    root.add_children([phase_explore, phase_select, phase_fetch])

    return root

# Petite classe utilitaire pour le Timer simple
class Delay(py_trees.behaviour.Behaviour):
    def __init__(self, name, duration):
        super().__init__(name)
        self.duration = duration
        self.start_time = None

    def initialise(self):
        self.start_time = time.time()

    def update(self):
        if time.time() - self.start_time > self.duration:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


import time


def main():
    rclpy.init()

    # Création du noeud ROS pour le superviseur
    node = Node("bt_supervisor")

    # Création de l'arbre
    root = create_tree(node)

    # Wrapper ROS pour l'arbre (gère le tick rate etc.)
    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=True  # Affiche l'arbre dans la console au démarrage
    )

    try:
        # Setup (appelle les méthodes .setup() de tous les behaviors)
        tree.setup(node=node, timeout=15.0)

        print(" démarrage de l'arbre... (CTRL+C pour quitter)")
        tree.visitors.append(py_trees.visitors.SnapshotVisitor())
        # Boucle de 'Tick' (Fréquence 10Hz)
        tree.tick_tock(period_ms=100)

        # Pour garder le noeud ROS actif (callbacks)
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("Arrêt demandé.")
        pass
    except Exception as e:
        print(f"Erreur critique: {e}")
    finally:
        tree.shutdown()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()