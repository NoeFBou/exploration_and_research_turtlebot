import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Chemins
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_tb3_nav2 = get_package_share_directory('turtlebot3_navigation2')
    pkg_project = get_package_share_directory('tb3_autonomy')

    # Configuration
    use_sim_time = True
    
    # 1. Lancer Gazebo avec ton monde .sdf
    # Note: Turtlebot3 utilise souvent 'empty_world.launch.py' et passe le world en argument
    # ou 'turtlebot3_world.launch.py'. Adaptons pour ton SDF.
    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_gazebo, 'launch', 'turtlebot3_world.launch.py')
        ),
        # On remplace le monde par défaut par le tien (assure-toi que le SDF est valide)
        # Si ton SDF ne charge pas le robot, il faudra utiliser spawn_entity séparément.
        # Pour simplifier ici, on suppose que ton SDF est un monde (murs) et le launch TB3 ajoute le robot.
        # Sinon, utilise simplement le launch file standard et change juste le chemin 'world'.
    )

    # 2. Navigation2 + SLAM (Simultaneous Localization and Mapping)
    # On met 'slam': 'True' pour que le robot cartographie en bougeant
    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_nav2, 'launch', 'navigation2.launch.py')
        ),
        launch_arguments={
            'use_sim_time': str(use_sim_time),
            'slam': 'True',
            'params_file': os.path.join(pkg_tb3_nav2, 'param', 'waffle.yaml') # ou burger.yaml
        }.items()
    )

    # 3. Explore Lite (L'algorithme frontier-based exploration)
    explore_cmd = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        output='screen',
        parameters=[{
            'robot_base_frame': 'base_link',
            'costmap_topic': '/map',
            'visualize': True,
            'planner_frequency': 0.33,
            'progress_timeout': 30.0,
            'potential_scale': 3.0,
            'orientation_scale': 0.0,
            'gain_scale': 1.0,
            'transform_tolerance': 0.3,
            'min_frontier_size': 0.75,
        }]
    )

    # 4. Ton Superviseur
    supervisor_cmd = Node(
        package='tb3_autonomy',
        executable='supervisor',
        name='mission_supervisor',
        output='screen',
        emulate_tty=True # Pour voir les print() et input() dans la console
    )

    return LaunchDescription([
        gazebo_cmd,
        nav2_cmd,
        explore_cmd,
        supervisor_cmd
    ])
