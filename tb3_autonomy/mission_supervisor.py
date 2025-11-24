# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator
import threading
import sys
import time

# Topic ou l'IA (ou un script test) publie la position de l'objet
TOPIC_DETECTION = '/object_detection/pose' 

class MissionSupervisor(Node):
    def __init__(self):
        super().__init__('mission_supervisor')
        
        # --- 1. Initialisation ---
        self.navigator = BasicNavigator()
        self.found_object_pose = None
        self.target_object_name = ""
        self.exploration_active = False
        self.mission_started = False
        
        self.create_subscription(PoseStamped, TOPIC_DETECTION, self.detection_callback, 10)
        
        # Publisher pince
        self.gripper_pub = self.create_publisher(String, '/gripper/command', 10)

    def detection_callback(self, msg):
        if self.exploration_active and self.found_object_pose is None:
            print(f"\n[SUPERVISOR] !!! OBJET DETECTE : {self.target_object_name} !!!", flush=True)
            self.get_logger().info(f"OBJET DETECTE : {self.target_object_name} !")
            self.found_object_pose = msg
            self.exploration_active = False # On arrete l'exploration
            self.navigator.cancelTask() # Stop immediat

    def run_console_menu(self):
        """Menu interactif dans un thread separe"""
        print("\n" + "="*40, flush=True)
        print("--- MISSION CONTROL ---", flush=True)
        print("1. Boule Rouge", flush=True)
        print("2. Carre Bleu", flush=True)
        print("="*40, flush=True)
        
        while not self.mission_started:
            try:
                # Python 3 input standard
                choice = input("Entrez le numero (1 ou 2) : ")
            except EOFError:
                continue

            if choice.strip() == '1':
                self.target_object_name = "boule_rouge"
                self.mission_started = True
            elif choice.strip() == '2':
                self.target_object_name = "carre_bleu"
                self.mission_started = True
            else:
                print(f"Choix '{choice}' invalide. Reessayez.", flush=True)
        
        print(f"\n[SUPERVISOR] CIBLE CONFIRMEE : {self.target_object_name}", flush=True)
        print("[SUPERVISOR] DEMARRAGE EXPLORATION...", flush=True)
        self.start_exploration()

    def start_exploration(self):
        self.exploration_active = True
        
        # NOTE : explore_lite tourne deja tout seul en fond.
        # Ici on surveille juste la detection.
        
        # Boucle de surveillance
        while rclpy.ok() and self.found_object_pose is None:
            # On attend juste que le callback detection se declenche
            # ou que l'utilisateur coupe (Ctrl+C)
            time.sleep(0.1) # Pause pour economiser le CPU

        if self.found_object_pose is not None:
             self.perform_approach_and_grasp()

    def perform_approach_and_grasp(self):
        print("[SUPERVISOR] Approche de l'objet...", flush=True)
        self.navigator.goToPose(self.found_object_pose)
        
        while not self.navigator.isTaskComplete():
            time.sleep(0.1) # On attend la fin du mouvement
        
        print("[SUPERVISOR] Action: Saisie...", flush=True)
        # Sequence pince
        self.gripper_pub.publish(String(data="OPEN"))
        
        # Petite pause simulee
        time.sleep(1.0)
        
        self.gripper_pub.publish(String(data="CLOSE"))
        print("[SUPERVISOR] Objet saisi.", flush=True)
        
        print("[SUPERVISOR] Retour a la base (0,0)...", flush=True)
        # Retour base
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.pose.position.x = 0.0
        initial_pose.pose.position.y = 0.0
        initial_pose.pose.orientation.w = 1.0 
        
        self.navigator.goToPose(initial_pose)
        while not self.navigator.isTaskComplete():
            time.sleep(0.1)

        print("[SUPERVISOR] MISSION TERMINEE !", flush=True)


def main():
    rclpy.init()
    supervisor = MissionSupervisor()

    # On lance le menu dans un thread parallele pour ne pas bloquer ROS
    menu_thread = threading.Thread(target=supervisor.run_console_menu)
    menu_thread.start()

    # On laisse ROS tourner sur le main thread
    try:
        rclpy.spin(supervisor)
    except KeyboardInterrupt:
        pass
    finally:
        supervisor.destroy_node()
        rclpy.shutdown()
        menu_thread.join()

if __name__ == '__main__':
    main()