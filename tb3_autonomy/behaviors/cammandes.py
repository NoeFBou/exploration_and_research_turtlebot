# class WaitForUserSelection(py_trees.behaviour.Behaviour):
#     def __init__(self, name="User Selection"):
#         super(WaitForUserSelection, self).__init__(name)
#         self.blackboard = py_trees.blackboard.Client(name="Interface")
#         self.blackboard.register_key(key="known_objects", access=py_trees.common.Access.READ)
#         # C'est ici qu'on écrira la cible choisie pour la suite
#         self.blackboard.register_key(key="target_pose_map", access=py_trees.common.Access.WRITE)
#
#     def update(self):
#         objects = self.blackboard.known_objects
#
#         if not objects:
#             print("Aucun objet trouvé pendant l'exploration !")
#             return py_trees.common.Status.FAILURE
#
#         print("\n" + "=" * 30)
#         print("LISTE DES OBJETS DÉTECTÉS :")
#         for obj in objects:
#             print(f"[{obj['id']}] X={obj['x']:.2f}, Y={obj['y']:.2f}")
#         print("=" * 30)
#
#         try:
#             choice = input("Entrez l'ID de l'objet à récupérer (ou 'q' pour quitter): ")
#             if choice.lower() == 'q':
#                 return py_trees.common.Status.FAILURE
#
#             target_id = int(choice)
#             # Retrouver l'objet dans la liste
#             selected_obj = next((o for o in objects if o['id'] == target_id), None)
#
#             if selected_obj:
#                 print(f"Cible #{target_id} sélectionnée. Démarrage récupération...")
#                 # On écrit la pose dans la variable que Nav2 et Actions utilisent
#                 self.blackboard.target_pose_map = selected_obj['pose']
#                 return py_trees.common.Status.SUCCESS
#             else:
#                 print("ID Invalide.")
#                 return py_trees.common.Status.RUNNING
#
#         except ValueError:
#             return py_trees.common.Status.RUNNING