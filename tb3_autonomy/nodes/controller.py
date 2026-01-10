#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool
from visualization_msgs.msg import MarkerArray
import threading
import time


class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')

        # Publisher pour envoyer l'ordre au robot
        self.pub_choice = self.create_publisher(Int32, '/mission/select_target', 10)
        self.pub_start = self.create_publisher(Bool, '/mission/start', 10)
        # Subscriber pour voir la liste des objets (publiée par le superviseur)
        self.sub_markers = self.create_subscription(
            MarkerArray,
            '/supervisor/known_objects',
            self.markers_cb,
            10
        )
        self.known_ids = []
        self.mission_started = False

        self.print_menu_timer = self.create_timer(2.0, self.display_loop)
        self.get_logger().info("--- CONTRÔLEUR DE MISSION ---")
        self.get_logger().info("Attente de données du superviseur...")

    def markers_cb(self, msg):
        # On extrait juste les IDs des marqueurs de type TEXTE
        current_ids = []
        for m in msg.markers:
            if m.ns == "ids":  # On filtre pour ne pas avoir les doublons (sphères)
                # Le texte est "ID X", on récupère X
                try:
                    obj_id = int(m.text.split()[-1])
                    current_ids.append(obj_id)
                except:
                    pass
        current_ids = []
        for m in msg.markers:
            if m.ns == "ids":
                try:
                    obj_id = int(m.text.split()[-1])
                    current_ids.append(obj_id)
                except:
                    pass
        current_ids.sort()
        self.known_ids = current_ids

    def display_loop(self):
        # Cette boucle tourne en fond, mais pour l'input propre,
        # on va le faire dans le main thread.
        pass

    def ask_user(self):
        while rclpy.ok():
            if not self.known_ids:
                print("En attente d'objets détectés...", end='\r')
                continue

            print("\n" + "=" * 30)
            print(" OBJETS DISPONIBLES :")
            for i in self.known_ids:
                print(f" -> Objet #{i}")
            print("=" * 30)

            try:
                raw = input("Entrez l'ID à récupérer : ")
                choice = int(raw)

                # Envoi de la commande
                msg = Int32()
                msg.data = choice
                self.pub_choice.publish(msg)
                print(f" >> Commande envoyée : {choice}")

            except ValueError:
                print("Erreur : Entrez un nombre entier.")

    def run_interface(self):
        # --- PHASE 1 : DÉMARRAGE ---
        print("\n" + "#" * 40)
        print("PRÊT À DÉMARRER LA MISSION")
        print("Vérifiez que l'IA est chargée et que Gazebo est prêt.")
        print("#" * 40)

        input(">>> Appuyez sur [ENTRÉE] pour lancer l'exploration...")

        msg = Bool()
        msg.data = True
        self.pub_start.publish(msg)
        print(">> Signal de départ envoyé !")
        self.mission_started = True

        # --- PHASE 2 : SÉLECTION (Votre code précédent) ---
        print("\nExploration en cours... (Patientez 60s)")

        while rclpy.ok():
            if not self.known_ids:
                # Petite animation d'attente
                for char in "|/-\\":
                    print(f"En attente d'objets détectés... {char}", end='\r')
                    time.sleep(0.1)
                continue

            print("\n" + "=" * 30)
            print(" OBJETS DISPONIBLES :")
            for i in self.known_ids:
                print(f" -> Objet #{i}")
            print("=" * 30)

            try:
                raw = input("Entrez l'ID à récupérer : ")
                choice = int(raw)

                msg_id = Int32()
                msg_id.data = choice
                self.pub_choice.publish(msg_id)
                print(f" >> Commande envoyée : Objet {choice}")
                break  # On quitte après avoir choisi, ou on peut rester pour changer d'avis

            except ValueError:
                print("Erreur : Entrez un nombre entier.")


def main(args=None):
    rclpy.init(args=args)
    node = MissionController()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.run_interface()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()