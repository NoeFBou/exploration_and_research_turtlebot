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
        self.pub_choice = self.create_publisher(Int32, '/mission/select_target', 10)
        self.pub_start = self.create_publisher(Bool, '/mission/start', 10)

        self.sub_markers = self.create_subscription(
            MarkerArray, '/supervisor/known_objects', self.markers_cb, 10
        )
        self.known_ids = []

    def markers_cb(self, msg):
        current_ids = []
        for m in msg.markers:
            # On filtre pour ne garder que les IDs (Texte)
            if m.ns == "ids":
                try:
                    # Le texte est "ID X (n=...)", on veut juste X
                    txt = m.text.split()[1] # "ID" est [0], le chiffre est [1]
                    obj_id = int(txt)
                    current_ids.append(obj_id)
                except: pass
        current_ids.sort()
        self.known_ids = current_ids

    def run_interface(self):
        # --- PHASE 1 : DÉMARRAGE ---
        print("\n" + "#"*40)
        print("PRÊT À DÉMARRER LA MISSION")
        print("#"*40)

        input(">>> Appuyez sur [ENTRÉE] pour lancer l'exploration...")

        self.pub_start.publish(Bool(data=True))
        print(">> Signal envoyé ! Exploration de 60s en cours...")

        # --- PHASE 2 : ATTENTE MANUELLE ---
        print("\n" + "!"*50)
        print("LA LISTE SE REMPLIT EN ARRIÈRE-PLAN.")
        print("Attendez la fin des 60s (ou quand vous voulez).")
        print("Appuyez sur [ENTRÉE] pour figer la liste et choisir.")
        print("!"*50)

        # Ce input() bloque le script, mais le thread ROS continue de mettre à jour known_ids
        input()

        # --- PHASE 3 : CHOIX ---
        while rclpy.ok():
            if not self.known_ids:
                print("Aucun objet trouvé pour l'instant... Attente...", end='\r')
                time.sleep(1.0)
                continue

            print("\n" + "="*30)
            print(f" LISTE FINALE ({len(self.known_ids)} objets) :")
            for i in self.known_ids:
                print(f" -> Objet #{i}")
            print("="*30)

            try:
                raw = input("Entrez l'ID à récupérer : ")
                choice = int(raw)

                if choice in self.known_ids:
                    self.pub_choice.publish(Int32(data=choice))
                    print(f" >> Cible #{choice} envoyée au robot !")
                    break
                else:
                    print(f"L'ID {choice} n'est pas dans la liste.")

            except ValueError:
                print("Erreur : Entrez un nombre entier.")

def main(args=None):
    rclpy.init(args=args)
    node = MissionController()

    # Thread séparé pour que les callbacks (mise à jour liste) tournent pendant le input()
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