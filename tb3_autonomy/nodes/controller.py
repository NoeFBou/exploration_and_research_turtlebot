#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool, String
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Twist
import threading
import time
import sys
import select
import termios
import tty
STEP_DURATION = 0.2
MOVE_BINDINGS = {
    'z': (0.15, 0.0),   # Avancer
    's': (-0.15, 0.0),  # Reculer
    'q': (0.0, 0.5),    # Gauche
    'd': (0.0, -0.5),   # Droite
    'x': (0.0, 0.0),    # Stop
}

class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')

        # Publishers
        self.pub_choice = self.create_publisher(Int32, '/mission/select_target', 10)
        self.pub_start = self.create_publisher(Bool, '/mission/start', 10)
        self.pub_confirm = self.create_publisher(Bool, '/mission/confirmation', 10)
        self.pub_abort = self.create_publisher(Bool, '/mission/abort', 10)
        self.pub_skip = self.create_publisher(Bool, '/mission/skip_nav', 10)

        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_catch = self.create_publisher(Bool, '/catch', 10)

        # Subscribers
        self.sub_markers = self.create_subscription(MarkerArray, '/supervisor/known_objects', self.markers_cb, 10)
        self.sub_status = self.create_subscription(String, '/mission/robot_status', self.status_cb, 10)
        self.settings = termios.tcgetattr(sys.stdin)
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
        elif msg.data == "WAITING_CATCH_VERIFICATION":
            self.robot_state = "READY_TO_VERIFY"
        elif msg.data == "MANUAL_RECOVERY":
            self.robot_state = "MANUAL_MODE"
        elif msg.data == "IDLE":
            self.robot_state = "IDLE"

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run_teleop_session(self):
        print("\n" + "#"*50)
        print("MODE MANUEL : PAS À PAS")
        print("#"*50)
        print(" [z]: Avancer (petit pas)")
        print(" [s]: Reculer (petit pas)")
        print(" [q]: Gauche  (petit angle)")
        print(" [d]: Droite  (petit angle)")
        print(" [ESPACE]: Ouvrir/Fermer Pince")
        print(" [ENTRÉE]: TERMINER et VÉRIFIER")
        print("#"*50)

        self.gripper_closed = False

        while rclpy.ok():
            key = self.get_key()

            # --- Mouvement Pas à Pas ---
            if key in MOVE_BINDINGS.keys():
                v, w = MOVE_BINDINGS[key]

                # 1. On envoie la vitesse
                twist = Twist()
                twist.linear.x = float(v)
                twist.angular.z = float(w)
                self.pub_cmd_vel.publish(twist)

                # 2. On attend un court instant (le "pas")
                # Si c'est 'x' (stop), on n'attend pas, on coupe direct
                if key != 'x':
                    time.sleep(STEP_DURATION)

                # 3. On force l'arrêt immédiatement après
                stop_twist = Twist() # Tout à 0.0 par défaut
                self.pub_cmd_vel.publish(stop_twist)

            # --- Pince ---
            elif key == ' ':
                self.gripper_closed = not self.gripper_closed
                state = "FERMÉE" if self.gripper_closed else "OUVERTE"
                print(f"\r Pince : {state}   ", end="", flush=True)
                self.pub_catch.publish(Bool(data=self.gripper_closed))

            # --- Validation / Sortie ---
            elif key == '\r' or key == '\n': # Touche Entrée
                # Arrêt moteur par sécurité
                self.pub_cmd_vel.publish(Twist())
                print("\n\n>>> Fin du pilotage.")

                while True:
                    resp = input("Avez-vous attrapé l'objet ? (o/n) : ").lower()
                    if resp in ['o', 'y', 'oui']:
                        print(">> SUCCÈS CONFIRMÉ. Retour à la base.")
                        for _ in range(10): # On envoie 10 fois le signal
                            self.pub_confirm.publish(Bool(data=True))
                            time.sleep(0.05)
                        #self.pub_confirm.publish(Bool(data=True))
                        self.robot_state = "MOVING"
                        return
                    elif resp in ['n', 'non']:
                        print(">> PAS ATTRAPÉ. On reprend le contrôle manuel...")
                        self.gripper_closed = False
                        self.pub_catch.publish(Bool(data=False))
                        print(">> Pince ouverte. C'est reparti !")
                        break
                    else:
                        print("Répondez par 'o' ou 'n'.")

            elif key == '\x03': # Ctrl+C
                break

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
                    print(f"\rEn attente d'objets... {len(self.known_ids)}", end="")
                    time.sleep(1)
                    continue

                try:
                    print(f"OBJETS DISPONIBLES : {self.known_ids}")
                    raw = input("Entrez l'ID à récupérer (ou 'wait' pour attendre) : ")
                    if self.robot_state != "IDLE": continue

                    if raw == 'wait': continue
                    choice = int(raw)
                    if choice in self.known_ids:
                        print(f" >> Cible #{choice} envoyée ! Le robot y va...")
                        self.pub_start.publish(Bool(data=True))
                        self.pub_choice.publish(Int32(data=choice))
                        self.robot_state = "MOVING"
                        self.wait_for_arrival()
                    else:
                        print("ID inconnu.")
                except ValueError:
                    print("Entrée invalide.")
                except: pass
            time.sleep(0.1)

    def wait_for_arrival(self):
        print("Déplacement en cours... [S]=Skip")

        while rclpy.ok():
            # 1. Gestion du SKIP (toujours actif)
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                line = sys.stdin.readline().strip()
                if line.lower() == 's':
                    print("\n>> COMMANDE SKIP ENVOYÉE !")
                    self.pub_skip.publish(Bool(data=True))

            # 2. CAS A : Robot arrivé, demande alignement
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

            # 3. CAS B : Demande prise (Auto)
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
                        print(">> Refus. Passage en mode manuel probable...")
                        self.robot_state = "MOVING"
                        break
                    else:
                        print("Répondez par 'o' ou 'n'.")

            # 4. CAS C : Vérification (Après Auto Catch)
            if self.robot_state == "READY_TO_VERIFY":
                print("\n" + "?"*40)
                print("L'OBJET EST-IL ATTRAPÉ CORRECTEMENT ?")
                print("?"*40)

                while True:
                    verif = input("Succès ? (o/n) : ").lower()
                    if verif in ['o', 'y', 'oui', 'yes']:
                        # On insiste un peu pour être sûr que le BT reçoive le message
                        for _ in range(5):
                            self.pub_confirm.publish(Bool(data=True))
                            time.sleep(0.05)
                        print(">> SUCCÈS CONFIRMÉ ! Le robot rentre à la base...")
                        self.robot_state = "MOVING"
                        # On ne return PAS ici, on attend que le robot repasse IDLE
                        break
                    elif verif in ['n', 'non', 'no']:
                        self.pub_confirm.publish(Bool(data=False))
                        print(">> ÉCHEC SIGNALÉ. Le robot lâche et réessaie...")
                        self.robot_state = "MOVING"
                        break
                    else:
                        print("Répondez par 'o' ou 'n'.")

            # 5. CAS D : Mode Manuel
            if self.robot_state == "MANUAL_MODE":
                self.run_teleop_session()
                # --- CORRECTION IMPORTANTE ---
                # On a supprimé le "return" qui était ici.
                # On continue la boucle while pour écouter ce que le BT veut faire ensuite.
                # (Par exemple : READY_TO_VERIFY ou IDLE)
                self.robot_state = "MOVING" # Reset état local pour éviter de relancer teleop en boucle

            # 6. Fin de mission (Le BT est revenu au début)
            if self.robot_state == "IDLE":
                print("\n>>> Mission terminée. Retour au menu principal.")
                return

            time.sleep(0.1)
# def wait_for_arrival(self):
#     print("Déplacement en cours... [S]=Skip")
#
#     while rclpy.ok():
#         # 1. Gestion du SKIP (toujours actif)
#         if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
#             line = sys.stdin.readline().strip()
#             if line.lower() == 's':
#                 print("\n>> COMMANDE SKIP ENVOYÉE !")
#                 self.pub_skip.publish(Bool(data=True))
#
#         # 2. CAS A : Robot arrivé, demande alignement
#         if self.robot_state == "READY_TO_ALIGN":
#             print("\n" + "!"*40)
#             print("ROBOT ARRIVÉ (Nav2).")
#             print("!"*40)
#
#             while True:
#                 q1 = input("Voulez-vous essayer d'attraper cet objet ? (o/n) : ").lower()
#                 if q1 in ['n', 'non', 'no']:
#                     print(">> ABANDON. Retour au menu.")
#                     self.pub_abort.publish(Bool(data=True))
#                     self.pub_confirm.publish(Bool(data=False))
#                     self.robot_state = "IDLE"
#                     return
#                 elif q1 in ['o', 'y', 'oui', 'yes']:
#                     print(">> OK, Alignement et Approche en cours...")
#                     self.pub_confirm.publish(Bool(data=True))
#                     self.robot_state = "MOVING"
#                     break
#                 else:
#                     print("Répondez par 'o' ou 'n'.")
#
#         # 3. CAS B : Demande prise (Auto)
#         if self.robot_state == "READY_TO_CATCH":
#             print("\n" + "!"*40)
#             print("ROBOT ALIGNÉ ET PROCHE (22cm).")
#             print("!"*40)
#
#             while True:
#                 resp = input("Pincer maintenant ? (o/n) : ").lower()
#                 if resp in ['o', 'y', 'oui', 'yes']:
#                     self.pub_confirm.publish(Bool(data=True))
#                     print(">> Catch envoyé ! Attente vérification...")
#                     self.robot_state = "MOVING"
#                     break
#                 elif resp in ['n', 'non', 'no']:
#                     self.pub_confirm.publish(Bool(data=False))
#                     print(">> Refus. Passage en mode manuel probable...")
#                     self.robot_state = "MOVING"
#                     break
#                 else:
#                     print("Répondez par 'o' ou 'n'.")
#
#         # 4. CAS C : Vérification (Après Auto Catch)
#         if self.robot_state == "READY_TO_VERIFY":
#             print("\n" + "?"*40)
#             print("L'OBJET EST-IL ATTRAPÉ CORRECTEMENT ?")
#             print("?"*40)
#
#             while True:
#                 verif = input("Succès ? (o/n) : ").lower()
#                 if verif in ['o', 'y', 'oui', 'yes']:
#                     # On insiste un peu pour être sûr que le BT reçoive le message
#                     for _ in range(5):
#                         self.pub_confirm.publish(Bool(data=True))
#                         time.sleep(0.05)
#                     print(">> SUCCÈS CONFIRMÉ ! Le robot rentre à la base...")
#                     self.robot_state = "MOVING"
#                     # On ne return PAS ici, on attend que le robot repasse IDLE
#                     break
#                 elif verif in ['n', 'non', 'no']:
#                     self.pub_confirm.publish(Bool(data=False))
#                     print(">> ÉCHEC SIGNALÉ. Le robot lâche et réessaie...")
#                     self.robot_state = "MOVING"
#                     break
#                 else:
#                     print("Répondez par 'o' ou 'n'.")
#
#         # 5. CAS D : Mode Manuel
#         if self.robot_state == "MANUAL_MODE":
#             self.run_teleop_session()
#             # --- CORRECTION IMPORTANTE ---
#             # On a supprimé le "return" qui était ici.
#             # On continue la boucle while pour écouter ce que le BT veut faire ensuite.
#             # (Par exemple : READY_TO_VERIFY ou IDLE)
#             self.robot_state = "MOVING" # Reset état local pour éviter de relancer teleop en boucle
#
#         # 6. Fin de mission (Le BT est revenu au début)
#         if self.robot_state == "IDLE":
#             print("\n>>> Mission terminée. Retour au menu principal.")
#             return
#
#         time.sleep(0.1)
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