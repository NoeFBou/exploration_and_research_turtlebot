import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # --- 0. Configuration ---
    use_sim_time = 'true'
    
    # Chemins
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_project = get_package_share_directory('tb3_autonomy')
    
    nav2_params_path = os.path.join(pkg_nav2_bringup, 'params', 'nav2_params.yaml')

    # Fix Graphique & Modèle (Toujours utile)
    env_gl = SetEnvironmentVariable(name='LIBGL_ALWAYS_SOFTWARE', value='1')
    env_lidar = SetEnvironmentVariable(name='LDS_MODEL', value='LDS-01')

    # --- 1. Simulation (Gazebo + Robot + Laser) ---
    # AU LIEU DE TOUT REFAIRE, ON APPELLE LE FICHIER OFFICIEL QUI MARCHE
    # Il va lancer Gazebo, charger le monde par défaut, spawner le robot ET activer le laser.
    tb3_world_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_gazebo, 'launch', 'turtlebot3_world.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'x_pose': '-2.0', # On peut surcharger la position ici !
            'y_pose': '-0.5'
        }.items()
    )

    # --- 2. Navigation & SLAM ---
    slam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'slam_launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time, 'params_file': nav2_params_path}.items()
    )

    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time, 
            'params_file': nav2_params_path,
            'log_level': 'warn'
        }.items()
    )

    # --- 3. Outils ---
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    explore_cmd = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        output='screen',
        parameters=[{'use_sim_time': True, 'visualize': True}]
    )

    supervisor_cmd = Node(
        package='tb3_autonomy',
        executable='supervisor',
        name='mission_supervisor',
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        env_gl, env_lidar,
        tb3_world_cmd,  # Le socle solide
        slam_cmd,
        navigation_cmd,
        rviz_cmd,
        explore_cmd,
        supervisor_cmd
    ])