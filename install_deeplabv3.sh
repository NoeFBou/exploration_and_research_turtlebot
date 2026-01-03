#!/bin/bash
# Script d'installation automatique de DeepLabV3+ pour tb3_autonomy
# Usage: ./install_deeplabv3.sh

set -e  # Arrêter en cas d'erreur

echo "======================================"
echo "Installation DeepLabV3+ pour OAK-D"
echo "======================================"
echo ""

# Couleurs pour le terminal
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Vérifier qu'on est dans le bon répertoire
if [ ! -f "setup.py" ]; then
    echo -e "${RED}❌ Erreur: setup.py introuvable${NC}"
    echo "Veuillez exécuter ce script depuis le répertoire tb3_autonomy"
    exit 1
fi

# Étape 1 : Installer les dépendances système
echo -e "${YELLOW}[1/6]${NC} Installation des dépendances système..."
sudo apt update
sudo apt install -y python3-pip udev

# Étape 2 : Règles udev pour OAK-D
echo -e "${YELLOW}[2/6]${NC} Configuration des règles udev pour OAK-D..."
if [ ! -f "/etc/udev/rules.d/80-movidius.rules" ]; then
    echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    echo -e "${GREEN}✓${NC} Règles udev installées"
else
    echo -e "${GREEN}✓${NC} Règles udev déjà présentes"
fi

# Étape 3 : Installer les dépendances Python
echo -e "${YELLOW}[3/6]${NC} Installation des dépendances Python..."
if [ -f "requirements.txt" ]; then
    pip3 install -r requirements.txt --break-system-packages
    echo -e "${GREEN}✓${NC} Dépendances Python installées"
else
    echo -e "${RED}❌ Fichier requirements.txt introuvable${NC}"
    exit 1
fi

# Étape 4 : Vérifier que segmentation_detector.py existe
echo -e "${YELLOW}[4/6]${NC} Vérification des fichiers..."
if [ ! -f "tb3_autonomy/segmentation_detector.py" ]; then
    echo -e "${RED}❌ Erreur: segmentation_detector.py introuvable dans tb3_autonomy/${NC}"
    echo "Veuillez copier le fichier avant de lancer ce script"
    exit 1
fi
echo -e "${GREEN}✓${NC} Fichiers présents"

# Étape 5 : Recompiler le package
echo -e "${YELLOW}[5/6]${NC} Compilation du package ROS2..."
cd ~/ros2_ws
colcon build --packages-select tb3_autonomy --symlink-install
source install/setup.bash
echo -e "${GREEN}✓${NC} Package compilé"

# Étape 6 : Test de la caméra OAK-D
echo -e "${YELLOW}[6/6]${NC} Test de connexion OAK-D..."
if lsusb | grep -q "03e7"; then
    echo -e "${GREEN}✓${NC} Caméra OAK-D détectée"
    
    # Télécharger le modèle en avance
    echo "Téléchargement du modèle DeepLabV3+..."
    python3 -c "import blobconverter; print('Modèle:', blobconverter.from_zoo('deeplab_v3_mnv2_256x256', zoo_type='depthai'))"
    echo -e "${GREEN}✓${NC} Modèle téléchargé"
else
    echo -e "${YELLOW}⚠${NC}  Caméra OAK-D non détectée"
    echo "    Veuillez brancher la caméra et relancer le script"
fi

echo ""
echo "======================================"
echo -e "${GREEN}✅ Installation terminée !${NC}"
echo "======================================"
echo ""
echo "Pour tester le détecteur :"
echo "  ros2 run tb3_autonomy segmentation_detector"
echo ""
echo "Pour lancer le système complet :"
echo "  ros2 launch tb3_autonomy auto_explore.launch.py"
echo ""
echo "Documentation complète : voir INSTALLATION_GUIDE.md"