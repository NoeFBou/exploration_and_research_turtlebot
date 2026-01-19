import os
import xacro
import re
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, GroupAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_prefix
from launch.actions import TimerAction

def generate_launch_description():
    # --- 1. CONFIGURATION GÉNÉRALE ---
    use_sim_time = 'true'

    # Noms de fichiers (Mise à jour pour Xacro)
    # Assure-toi que ce fichier existe bien dans tb3_autonomy/urdf/
    xacro_file_name = 'turtlebot3_burger_gripper.urdf.xacro'

    # Récupération des dossiers des packages
    pkg_tb3_autonomy = get_package_share_directory('tb3_autonomy')
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_stereo_image_proc = get_package_share_directory('stereo_image_proc')
    pkg_tb3_description = get_package_share_directory('turtlebot3_description')

    # --- 2. GESTION DES CHEMINS ET VARIABLES D'ENVIRONNEMENT ---

    # Configuration des modèles Gazebo
    existing = os.environ.get('GAZEBO_MODEL_PATH', '')
    gazebo_model_path = existing + ':' + os.path.join(pkg_tb3_description, '..') + ':' + os.path.join(pkg_tb3_autonomy, '..')

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=gazebo_model_path
    )

    env_lidar = SetEnvironmentVariable(name='LDS_MODEL', value='LDS-01')
    env_gl = SetEnvironmentVariable(name='LIBGL_ALWAYS_SOFTWARE', value='0')

    # Chemins vers le Xacro et le fichier de config des contrôleurs
    urdf_path = os.path.join(pkg_tb3_autonomy, 'urdf', xacro_file_name)
    controllers_yaml = os.path.join(pkg_tb3_autonomy, 'params', 'ros2_controllers.yaml')


    # --- 3. TRAITEMENT DU XACRO ---
    # On traite le fichier Xacro pour obtenir le XML final du robot
    # On passe le chemin du yaml si ton xacro utilise $(arg controllers_file)
    doc = xacro.process_file(
        urdf_path,
        mappings={'controllers_file': controllers_yaml}
    )
    robot_desc = doc.toxml()
    robot_desc = re.sub(r'<!--(.|\n)*?-->', '', robot_desc)
    robot_desc = re.sub(r'<\?xml.*?\?>', '', robot_desc).strip()
    # --- 4. NODES PRINCIPAUX ---

    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_desc
        }]
    )

    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_custom_waffle',
            '-x', '-2.0',
            '-y', '-0.5',
            '-z', '0.05'
        ],
        output='screen'
    )

    # --- 5. CONTROLLERS (ROS2 CONTROL) ---
    # Ces nœuds chargent les contrôleurs définis dans ton YAML

    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120'
        ],
        output='screen'
    )

    gripper_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'gripper_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120'
        ],
        output='screen'
    )


# CRUCIAL : On attend que spawn_entity_cmd se termine (que le robot soit dans Gazebo)
    # avant de lancer les contrôleurs. Sinon, ils ne trouvent pas l'interface matérielle.
    spawn_controllers_after_robot = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity_cmd,
            on_exit=[jsb_spawner, gripper_spawner]
        )
    )

    # --- 6. GAZEBO ET MONDE ---

    world_path = os.path.join(pkg_tb3_autonomy, 'worlds', 'my_room.sdf')

    # gazebo_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
    #     ),
    #     launch_arguments={'world': world_path}.items()
    # )
    gazebo_ros_prefix = get_package_prefix('gazebo_ros')   # typiquement /opt/ros/humble
    ros_lib = os.path.join(gazebo_ros_prefix, 'lib')

    init_plugin = os.path.join(ros_lib, 'libgazebo_ros_init.so')
    factory_plugin = os.path.join(ros_lib, 'libgazebo_ros_factory.so')

    gzserver_cmd = ExecuteProcess(
        cmd=[
            'gzserver', '--verbose', world_path,
            '-s', init_plugin,
            '-s', factory_plugin,
        ],
        output='log'
    )


    gzclient_cmd = ExecuteProcess(
            cmd=['gzclient'],
            output='log'
        )
    # --- 7. NAVIGATION ET EXPLORATION ---

    nav2_params = os.path.join(pkg_tb3_autonomy, 'params', 'my_nav2_params.yaml')

    slam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'slam_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
            'transform_timeout': '0.5' # Parfois utile d'augmenter si la simulation lag
        }.items()
    )

    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
            'log_level': 'Fatal'
        }.items()
    )

    explore_cmd = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_base_frame': 'base_link',
            'costmap_topic': '/map',
            'visualize': True,
            'planner_frequency': 0.2,
            'progress_timeout': 120.0,
            'min_frontier_size': 0.30,
            'potential_scale': 3.0,
            'gain_scale': 1.0,
            'transform_tolerance': 0.5,
        }]

        # parameters=[{
        #     'use_sim_time': True,
        #     'robot_base_frame': 'base_link',
        #     'costmap_topic': '/map',
        #     'visualize': True,
        #     'planner_frequency': 0.33,
        #     'progress_timeout': 30.0,
        #     'potential_scale': 3.0,
        #     'orientation_scale': 0.0,
        #     'gain_scale': 1.0,
        #     'transform_tolerance': 0.3,
        #     'min_frontier_size': 0.75,
        # }]
    )
    explore_delayed = TimerAction(period=12.0, actions=[explore_cmd])

    # --- 8. AUTRES NODES (STEREO, RVIZ, ETC.) ---

    stereo_proc = GroupAction(
        actions=[
            PushRosNamespace('oakd'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_stereo_image_proc, 'launch', 'stereo_image_proc.launch.py')
                ),
                launch_arguments={
                    'approximate_sync': 'True',
                    'use_sim_time': 'True',
                    'left_namespace': 'left',
                    'right_namespace': 'right',
                }.items()
            )
        ]
    )

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': True}],
        output='log'
    )
    ai_node = Node(
        package='tb3_autonomy',
        executable='sim_yolo_depth',
        name='ai_node',
        output='screen',
        parameters=[{
            'weights': 'src/exploration_and_research_turtlebot/tb3_autonomy/yolo11n_red_cube.pt',
            'device': 'cpu',
            'conf': 0.5,
            'debug_view': True
        }]
    )

    catch_cmd = Node(
        package='tb3_autonomy',
        executable='catch_node',
        name='catch_node',
        output='screen',
        parameters=[{'use_sim_time': True, 'mode': 'sim'}],  # ou 'auto'
    )
    spawn_entity_delayed = TimerAction(
        period=2.0,
        actions=[spawn_entity_cmd]
    )
    bt_supervisor_cmd = Node(
        package='tb3_autonomy',
        executable='bt_supervisor',
        name='bt_supervisor',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    start_logic_after_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=gripper_spawner,
            on_exit=[bt_supervisor_cmd,catch_cmd, ai_node, explore_delayed]
        )
    )



    # --- 9. RETURN ---
    return LaunchDescription([
        # Environnement
        set_gazebo_model_path,
        env_lidar,
        env_gl,

        # Simu & Robot
        gzserver_cmd,
        gzclient_cmd,
        robot_state_publisher_cmd,
        spawn_entity_delayed , #spawn_entity_cmd,
        spawn_controllers_after_robot,
        # Navigation Stack
        slam_cmd,
        navigation_cmd,
        #explore_delayed,

        # Outils & Logique custom
        rviz_cmd,
        stereo_proc,
        #catch_cmd
        start_logic_after_controllers
    ])