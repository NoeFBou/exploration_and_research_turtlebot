# Projet TB3

## Prérequis Système

* **OS :** Ubuntu 22.04 LTS (Jammy Jellyfish)
* **ROS Version :** ROS 2 Humble

* ## Installation

### 1. Installer ROS 2 Humble
Si ROS 2 n'est pas encore installé, ouvrez un terminal et exécutez :

```bash
locale # assurez-vous d'avoir UTF-8
sudo apt update && sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL [https://raw.githubusercontent.com/ros/rosdistro/master/ros.key](https://raw.githubusercontent.com/ros/rosdistro/master/ros.key) -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] [http://packages.ros.org/ros2/ubuntu](http://packages.ros.org/ros2/ubuntu) $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop -y
sudo apt install python3-colcon-common-extensions -y
```

### 2. Installer les dépendances du projet
```bash
sudo apt install python3-pip -y
sudo apt install python3-opencv -y
sudo apt install ros-humble-cv-bridge -y
sudo apt install ros-humble-vision-opencv -y
```

### 3. Création de l'espace de travail et clonage
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
https://github.com/NoeFBou/exploration_and_research_turtlebot.git
git clone [https://github.com/NoeFBou/exploration_and_research_turtlebot.git](https://github.com/NoeFBou/exploration_and_research_turtlebot.git)
```

## Compilation
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```


