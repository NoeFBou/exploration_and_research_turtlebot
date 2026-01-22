# README Nathan - Modifications et ajouts

## Modifications effectuees

### 1. Nouveau node : `sim_yolo_depth_node.py`

Ajout d'un node de detection YOLO + Depth adapte pour la simulation Gazebo.

**Fichier :** `tb3_autonomy/sim_yolo_depth_node.py`

**Probleme resolu :**
Les topics camera dans l'URDF (`/oakd/rgb/...`, `/oakd/depth/...`) ne correspondaient pas aux topics utilises dans l'ancien `object_detector.py` (`/rgb_camera/...`, `/depth_camera/...`).

**Fonctionnalites :**
- Subscribe aux bons topics Gazebo :
  - `/oakd/rgb/image_raw`
  - `/oakd/depth/image_raw`
  - `/oakd/rgb/camera_info`
- Synchronisation RGB + Depth avec `message_filters.ApproximateTimeSynchronizer`
- Detection YOLO sur GPU (ou CPU en fallback)
- Calcul de profondeur par mediane dans une ROI au centre de la bounding box
- Publication sur `/target_object_pose` (compatible avec `supervisor_node.py`)
- Fenetre de debug optionnelle avec visualisation des detections

**Parametres ROS :**
| Parametre | Default | Description |
|-----------|---------|-------------|
| `weights` | `best.pt` | Chemin vers le modele YOLO finetune |
| `device` | `0` | Device CUDA (0, 1, ...) ou `"cpu"` |
| `conf` | `0.5` | Seuil de confiance YOLO |
| `max_rate_hz` | `15.0` | Frequence max de traitement |
| `debug_view` | `True` | Afficher la fenetre OpenCV |

### 2. Nouveau node : `oak_yolo_depth_node.py`

Ajout d'un node de detection YOLO + Depth pour le **ROBOT REEL** avec camera OAK-D Pro physique (DepthAI).

**Fichier :** `tb3_autonomy/oak_yolo_depth_node.py`

**Fonctionnalites :**
- Utilise l'API DepthAI pour communiquer directement avec la camera OAK-D Pro
- Inference YOLO via un blob OpenVINO (tourne sur le VPU de la camera)
- Stereo depth aligne sur la camera RGB
- Calcul de profondeur par mediane dans une ROI
- Publication sur `/target_object_pose` (compatible avec `supervisor_node.py`)
- Thread de capture separe pour ne pas bloquer ROS

**Parametres ROS :**
| Parametre | Default | Description |
|-----------|---------|-------------|
| `blob_path` | `models/red_cube_01/best_openvino_2022.1_6shave.blob` | Chemin vers le blob YOLO |
| `camera_frame` | `oak_d_pro_color_optical_frame` | Frame TF de la camera |
| `class_name` | `red_cube` | Nom de la classe detectee |
| `conf_thres` | `0.5` | Seuil de confiance |
| `iou_thres` | `0.4` | Seuil IoU pour NMS |
| `input_size` | `640` | Taille d'entree YOLO |
| `depth_out_w` | `640` | Largeur sortie depth |
| `depth_out_h` | `360` | Hauteur sortie depth |
| `min_z` | `0.10` | Distance min (m) |
| `max_z` | `3.00` | Distance max (m) |
| `debug_view` | `True` | Afficher la fenetre OpenCV |

### 3. Mise a jour de `setup.py`

Ajout des entrypoints pour les nouveaux nodes :
```python
'sim_yolo_depth = tb3_autonomy.sim_yolo_depth_node:main',
'oak_yolo_depth = tb3_autonomy.oak_yolo_depth_node:main',
```

---

## Installation des dependances

### DepthAI (pour robot reel avec OAK-D)
```bash
pip install depthai opencv-python numpy
```

### Ultralytics (YOLO - pour simulation)
```bash
pip install ultralytics
```

### PyTorch avec CUDA (pour GPU - simulation)
```bash
# Verifier la version CUDA installee
nvcc --version

# Installer PyTorch avec CUDA (exemple pour CUDA 11.8)
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu118

# Ou pour CUDA 12.1
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121
```

### Verifier l'installation GPU
```python
import torch
print(torch.cuda.is_available())  # Doit afficher True
print(torch.cuda.get_device_name(0))  # Nom du GPU
```

---

## Test des nodes

### 1. Rebuild le package
```bash
cd ~/ros2_ws
# ou: cd ~/Documents/GitHub/exploration_and_research_turtlebot
colcon build --symlink-install
source install/setup.bash
```

### 2. Test en SIMULATION (Gazebo)

#### Lancer la simulation Gazebo
```bash
ros2 launch tb3_autonomy auto_explore.launch.py
```

#### Lancer le node de detection simulation (dans un autre terminal)
```bash
source install/setup.bash

ros2 run tb3_autonomy sim_yolo_depth --ros-args \
    -p weights:=/chemin/vers/best.pt \
    -p device:=0 \
    -p conf:=0.5 \
    -p debug_view:=true
```

### 3. Test sur ROBOT REEL (OAK-D Pro)

#### Lancer le node de detection robot reel
```bash
source install/setup.bash

ros2 run tb3_autonomy oak_yolo_depth --ros-args \
    -p blob_path:=/chemin/vers/best_openvino_2022.1_6shave.blob \
    -p camera_frame:=oak_d_pro_color_optical_frame \
    -p conf_thres:=0.5 \
    -p debug_view:=true
```

### 4. Verifier les topics
```bash
# Lister les topics disponibles
ros2 topic list | grep oakd

# Verifier que le node publie bien
ros2 topic echo /target_object_pose
```

---

## Architecture des topics camera

### Topics definis dans l'URDF (Gazebo publie)
| Capteur | Topic |
|---------|-------|
| RGB Image | `/oakd/rgb/image_raw` |
| RGB Info | `/oakd/rgb/camera_info` |
| Depth Image | `/oakd/depth/image_raw` |
| Depth Info | `/oakd/depth/camera_info` |

### Topic de sortie
| Node | Topic publie | Type |
|------|--------------|------|
| `sim_yolo_depth` | `/target_object_pose` | `geometry_msgs/PoseStamped` |
| `oak_yolo_depth` | `/target_object_pose` | `geometry_msgs/PoseStamped` |

---

## Piege connu : Crop du preview OAK-D (robot reel)

### Le probleme

Le `ColorCamera.preview` de DepthAI est **croppe par defaut** pour obtenir l'aspect ratio demande (ici 1:1 = 640x640). Cela signifie que les coordonnees `(u,v)` dans l'image carree du NN ne correspondent pas a un simple "scale" vers l'image depth (640x360, ratio 16:9).

**Consequence :**
- Au centre de l'image : ca fonctionne correctement
- Sur les bords : la ROI depth peut etre decalee, causant :
  - Distance Z = 0 (invalide)
  - X lateral biaise → le robot tourne trop ou pas assez

### Comment detecter ce probleme

Tester avec l'objet cible :
1. **Objet au centre** : le robot doit approcher correctement
2. **Objet tres a gauche/droite** : si la profondeur ou X deviennent incoherents, c'est le crop

### Fix applique dans le code

On utilise les **coordonnees depth (du,dv)** avec les **intrinsics correspondant a la resolution depth** :

```python
# Intrinsics pour la resolution depth (pas input_size)
K = calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, depth_out_w, depth_out_h)

# Calcul 3D avec coordonnees depth
X = (du - cx) * z / fx
Y = (dv - cy) * z / fy
```

### Fix supplementaire si probleme persiste

**Option simple** - Desactiver le crop du preview (au prix d'un etirement) :
```python
cam_rgb.setPreviewKeepAspectRatio(False)
```

**Option propre** - Utiliser un letterbox via `ImageManip` pour garder le FOV sans etirer (plus complexe a implementer).

---

## Notes

- Le node `object_detector.py` (detection par couleur HSV) reste disponible mais utilise les anciens topics. Il faudrait le mettre a jour si on veut l'utiliser avec la simulation actuelle.
- Le `supervisor_node.py` n'a pas besoin de modification car il subscribe deja a `/target_object_pose`.
