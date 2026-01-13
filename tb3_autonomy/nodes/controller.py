#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool, String
from visualization_msgs.msg import MarkerArray
import threading
import time
import sys
import select

class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')

        # Publishers
        self.pub_choice = self.create_publisher(Int32, '/mission/select_target', 10)
        self.pub_start = self.create_publisher(Bool, '/mission/start', 10)
        self.pub_confirm = self.create_publisher(Bool, '/mission/confirmation', 10)

        # Subscribers
        self.sub_markers = self.create_subscription(MarkerArray, '/supervisor/known_objects', self.markers_cb, 10)
        self.sub_status = self.create_subscription(String, '/mission/robot_status', self.status_cb, 10)
        self.pub_skip = self.create_publisher(Bool, '/mission/skip_nav', 10)
        self.known_ids = []
        self.robot_state = "IDLE"
        self.waiting_for_input = False

    def markers_cb(self, msg):
        current_ids = []
        for m in msg.markers:
            if m.ns == "ids":
                try:
                    txt = m.text.split()[1]
                    obj_id = int(txt)
                    current_ids.append(obj_id)
                except: pass
        current_ids.sort()
        self.known_ids = current_ids

    def status_cb(self, msg):
        if msg.data == "WAITING_CONFIRMATION":
            self.robot_state = "ARRIVED"

    def run_interface(self):
        # --- PHASE 1 : DÉMARRAGE ---
        print("\n" + "#"*40)
        print("PRÊT À DÉMARRER LA MISSION")
        print("#"*40)
        input(">>> Appuyez sur [ENTRÉE] pour lancer l'exploration...")

        self.pub_start.publish(Bool(data=True))
        print(">> Signal envoyé ! Exploration en cours...")

        # --- BOUCLE PRINCIPALE ---
        while rclpy.ok():

            # CAS A : On doit choisir un objet (Le robot attend au menu)
            # On détecte cela si on n'est PAS en train d'attendre une validation d'arrivée
            if self.robot_state != "ARRIVED":
                print("\n" + "="*40)
                print("MENU DE SÉLECTION (Appuyez sur ENTRÉE pour rafraîchir)")
                print("="*40)
                input() # Pause pour laisser le temps aux logs d'arriver ou juste attendre

                if not self.known_ids:
                    print("Aucun objet détecté pour l'instant.")
                    continue

                print(f"OBJETS DISPONIBLES : {self.known_ids}")
                raw = input("Entrez l'ID à récupérer (ou 'wait' pour attendre) : ")

                if raw.lower() == 'wait':
                    continue

                try:
                    choice = int(raw)
                    if choice in self.known_ids:
                        self.pub_start.publish(Bool(data=True))
                        time.sleep(0.2)

                        self.pub_choice.publish(Int32(data=choice))
                        print(f" >> Cible #{choice} envoyée ! Le robot y va...")
                        self.robot_state = "MOVING"
                        self.wait_for_arrival()
                    else:
                        print("ID inconnu.")
                except ValueError:
                    print("Entrée invalide.")

    def wait_for_arrival(self):
        """
        Attend que le robot arrive OU que l'utilisateur appuie sur 's'.
        """
        print("Déplacement en cours...")
        print(" [S] + ENTRÉE : Pour forcer l'arrêt (Skip Nav)")
        print(" (Surveillez Gazebo)")

        while rclpy.ok():
            # 1. Vérification arrivée Robot
            if self.robot_state == "ARRIVED":
                print("\n" + "!"*40)
                print("LE ROBOT EST ARRIVÉ (Confirmé par le Superviseur) !")
                print("!"*40)
                self.ask_catch() # On lance le dialogue
                return

                # 2. Vérification Entrée Utilisateur (Non-bloquant)
            # select vérifie si des données sont prêtes à être lues sur stdin
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                line = sys.stdin.readline().strip()
                if line.lower() == 's':
                    print("\n>> COMMANDE SKIP ENVOYÉE !")
                    self.pub_skip.publish(Bool(data=True))
                    # On n'attend pas 'ARRIVED', on suppose que le robot va s'arrêter
                    # et passer à la suite (Validation)
                    # Mais pour être propre, on continue d'attendre que le superviseur
                    # passe à l'état "WAITING_CONFIRMATION"
                else:
                    print("Touche ignorée. Tapez 's' pour passer.")

            time.sleep(0.1)

    def ask_catch(self):
        """Dialogue de validation séparé pour la clarté"""
        while True:
            resp = input("Voulez-vous attraper cet objet ? (o/n) : ").lower()
            if resp in ['o', 'y', 'oui', 'yes']:
                self.pub_confirm.publish(Bool(data=True))
                print(">> Catch en cours !")
                self.robot_state = "IDLE"
                return
            elif resp in ['n', 'non', 'no']:
                self.pub_confirm.publish(Bool(data=False))
                print(">> Annulation. Retour au menu...")
                self.robot_state = "IDLE"
                return
            else:
                print("Répondez par 'o' ou 'n'.")

def main(args=None):
    rclpy.init(args=args)
    node = MissionController()

    # Thread daemon pour gérer les callbacks ROS en arrière-plan
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.run_interface()
    except KeyboardInterrupt:
        print("\nArrêt du contrôleur.")
    except Exception as e:
        print(f"Erreur: {e}")
    finally:
        # Pas de destroy_node explicite ici pour éviter le crash 'terminate called'
        # rclpy.shutdown() suffira car le thread est daemon
        rclpy.shutdown()

if __name__ == '__main__':
    main()