import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file pour le PC (Workstation).
    Gère: Nav2, SLAM, Behavior Tree, RViz.

    """

    # --- Paths ---
    pkg_tb3_autonomy = get_package_share_directory('tb3_autonomy')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    nav2_params = os.path.join(pkg_tb3_autonomy, 'params', 'my_nav2_params.yaml')
    rviz_config = os.path.join(pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz')

    # --- 1. SLAM (Cartographie en temps réel) ---
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'slam_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_params
        }.items()
    )

    # --- 2. Navigation (Path Planning) ---
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',  # Réel
            'params_file': nav2_params,
            'map_subscribe_transient_local': 'true'
        }.items()
    )

    # --- 3. Superviseur (Behavior Tree) ---
    bt_supervisor = Node(
        package='tb3_autonomy',
        executable='bt_supervisor',
        name='bt_supervisor',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # --- 4. Exploration (Explore Lite) ---
    explore_node = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'robot_base_frame': 'base_link',
            'costmap_topic': '/map',
            'visualize': True,
            'planner_frequency': 0.33,
            'progress_timeout': 30.0,
            'potential_scale': 3.0,
            'orientation_scale': 0.0,
            'gain_scale': 1.0,
            'transform_tolerance': 0.3,
            'min_frontier_size': 0.5,
        }]
    )
    explore_delayed = TimerAction(period=5.0, actions=[explore_node])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}],
        output='log'
    )

    return LaunchDescription([
        slam_launch,
        navigation_launch,
        bt_supervisor,
        explore_delayed,
        rviz_node
    ])