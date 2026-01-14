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
        self.pub_abort = self.create_publisher(Bool, '/mission/abort', 10)
        self.pub_skip = self.create_publisher(Bool, '/mission/skip_nav', 10)

        # Subscribers
        self.sub_markers = self.create_subscription(MarkerArray, '/supervisor/known_objects', self.markers_cb, 10)
        self.sub_status = self.create_subscription(String, '/mission/robot_status', self.status_cb, 10)

        self.known_ids = []
        self.robot_state = "IDLE"

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
        # Mise à jour des statuts selon ce que l'arbre envoie
        if msg.data == "WAITING_ALIGNMENT":
            self.robot_state = "READY_TO_ALIGN"
        elif msg.data == "WAITING_CATCH" or msg.data == "WAITING_CONFIRMATION":
            self.robot_state = "READY_TO_CATCH"

        # === C'EST ICI QUE LE ROBOT VOUS ATTEND ===
        elif msg.data == "WAITING_CATCH_VERIFICATION":
            self.robot_state = "READY_TO_VERIFY"
        # ==========================================

        elif msg.data == "IDLE":
            self.robot_state = "IDLE"

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

            # Affiche le menu seulement si le robot est libre (IDLE)
            if self.robot_state == "IDLE":
                print("\n" + "="*40)
                print("MENU DE SÉLECTION (Appuyez sur ENTRÉE pour rafraîchir)")
                print("="*40)
                input() # Pause attente

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

                        self.robot_state = "MOVING" # On verrouille le menu
                        self.wait_for_arrival()     # On passe en mode surveillance
                    else:
                        print("ID inconnu.")
                except ValueError:
                    print("Entrée invalide.")

            time.sleep(0.1)

    def wait_for_arrival(self):
        print("Déplacement en cours... [S]=Skip")

        while rclpy.ok():
            # 1. Gestion du SKIP
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                line = sys.stdin.readline().strip()
                if line.lower() == 's':
                    print("\n>> COMMANDE SKIP ENVOYÉE !")
                    self.pub_skip.publish(Bool(data=True))

            # 2. CAS A : Le robot est arrivé (Nav2) et demande s'il peut s'aligner
            if self.robot_state == "READY_TO_ALIGN":
                print("\n" + "!"*40)
                print("ROBOT ARRIVÉ (Nav2).")
                print("!"*40)

                while True:
                    q1 = input("Voulez-vous essayer d'attraper cet objet ? (o/n) : ").lower()
                    if q1 in ['n', 'non', 'no']:
                        print(">> ABANDON. Retour au menu.")
                        self.pub_abort.publish(Bool(data=True))
                        self.pub_confirm.publish(Bool(data=False))
                        self.robot_state = "IDLE"
                        return
                    elif q1 in ['o', 'y', 'oui', 'yes']:
                        print(">> OK, Alignement et Approche en cours...")
                        self.pub_confirm.publish(Bool(data=True))
                        self.robot_state = "MOVING"
                        break
                    else:
                        print("Répondez par 'o' ou 'n'.")

            # 3. CAS B : Le robot s'est aligné/approché et demande s'il peut pincer
            if self.robot_state == "READY_TO_CATCH":
                print("\n" + "!"*40)
                print("ROBOT ALIGNÉ ET PROCHE (22cm).")
                print("!"*40)

                while True:
                    resp = input("Pincer maintenant ? (o/n) : ").lower()
                    if resp in ['o', 'y', 'oui', 'yes']:
                        self.pub_confirm.publish(Bool(data=True))
                        print(">> Catch envoyé ! Attente vérification...")
                        self.robot_state = "MOVING"
                        break
                    elif resp in ['n', 'non', 'no']:
                        self.pub_confirm.publish(Bool(data=False))
                        print(">> Refus. Le robot va reculer et réessayer...")
                        self.robot_state = "MOVING"
                        break
                    else:
                        print("Répondez par 'o' ou 'n'.")

            # === 4. CAS C : VÉRIFICATION DU SUCCÈS (C'est ce qui manquait !) ===
            if self.robot_state == "READY_TO_VERIFY":
                print("\n" + "?"*40)
                print("L'OBJET EST-IL ATTRAPÉ CORRECTEMENT ?")
                print("?"*40)

                while True:
                    verif = input("Succès ? (o/n) : ").lower()
                    if verif in ['o', 'y', 'oui', 'yes']:
                        self.pub_confirm.publish(Bool(data=True))
                        print(">> SUCCÈS CONFIRMÉ ! Le robot rentre à la base...")
                        self.robot_state = "MOVING" # Il rentre, on attend IDLE
                        break
                    elif verif in ['n', 'non', 'no']:
                        self.pub_confirm.publish(Bool(data=False))
                        print(">> ÉCHEC SIGNALÉ. Le robot lâche, recule et réessaie...")
                        self.robot_state = "MOVING" # Retour à la case départ (Alignement)
                        break
                    else:
                        print("Répondez par 'o' ou 'n'.")
            # ===================================================================

            # 5. CAS D : Retour Menu
            if self.robot_state == "IDLE":
                print("\n>>> Retour au menu principal (Robot prêt).")
                return

            time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.run_interface()
    except KeyboardInterrupt:
        print("\nArrêt du contrôleur.")
    except Exception as e:
        print(f"Erreur: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()