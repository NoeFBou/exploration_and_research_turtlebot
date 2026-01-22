import os
import re
import xacro

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    GroupAction,
    RegisterEventHandler,
    ExecuteProcess,
    TimerAction
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description() -> LaunchDescription:
    """
    Generates the complete LaunchDescription for the TurtleBot3 Autonomy mission.

    This launch file orchestrates:
    1. Gazebo simulation environment setup.
    2. Robot State Publisher (URDF/Xacro processing).
    3. Spawning the entity into Gazebo.
    4. ROS 2 Controllers (Joint State Broadcaster & Gripper Controller).
    5. Navigation Stack (SLAM Toolbox + Nav2).
    6. Autonomous Exploration (explore_lite).
    7. Vision & AI Nodes (Stereo Proc, YOLO simulation).
    8. Behavior Tree Supervisor & Custom Logic.
    """

    # =========================================================================
    # 1. CONFIGURATION & PATHS
    # =========================================================================
    use_sim_time = 'true'
    xacro_file_name = 'turtlebot3_burger_gripper.urdf.xacro'
    world_file_name = 'my_room.sdf'

    # Package Directories
    pkg_tb3_autonomy = get_package_share_directory('tb3_autonomy')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_stereo_image_proc = get_package_share_directory('stereo_image_proc')
    pkg_tb3_description = get_package_share_directory('turtlebot3_description')
    gazebo_ros_prefix = get_package_prefix('gazebo_ros')

    # File Paths
    urdf_path = os.path.join(pkg_tb3_autonomy, 'urdf', xacro_file_name)
    controllers_yaml = os.path.join(pkg_tb3_autonomy, 'params', 'ros2_controllers.yaml')
    nav2_params = os.path.join(pkg_tb3_autonomy, 'params', 'my_nav2_params.yaml')
    world_path = os.path.join(pkg_tb3_autonomy, 'worlds', world_file_name)

    # Dynamic Path for YOLO Model (Recommended over hardcoded 'src/...')
    # Assuming the model is installed into the share directory via setup.py/CMakeLists
    yolo_model_path = os.path.join(pkg_tb3_autonomy, 'models', 'yolo11n_red_cube.pt')
    # Fallback if file is not found in share (e.g. during development)
    if not os.path.exists(yolo_model_path):
        # Try local path relative to workspace root if needed, or keep your hardcoded one
        yolo_model_path = 'src/exploration_and_research_turtlebot/tb3_autonomy/yolo11n_red_cube.pt'

    # =========================================================================
    # 2. ENVIRONMENT VARIABLES
    # =========================================================================

    # Configure Gazebo Model Path to include TurtleBot3 models
    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    gazebo_model_path = f"{existing_model_path}:{os.path.join(pkg_tb3_description, '..')}:{os.path.join(pkg_tb3_autonomy, '..')}"

    set_env_vars = [
        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=gazebo_model_path),
        SetEnvironmentVariable(name='LDS_MODEL', value='LDS-01'),
        SetEnvironmentVariable(name='LIBGL_ALWAYS_SOFTWARE', value='0')
    ]

    # =========================================================================
    # 3. ROBOT DESCRIPTION (XACRO PROCESSING)
    # =========================================================================

    doc = xacro.process_file(urdf_path, mappings={'controllers_file': controllers_yaml})
    robot_desc = doc.toxml()

    # Clean up XML (remove comments and XML declaration)
    robot_desc = re.sub(r'', '', robot_desc)
    robot_desc = re.sub(r'<\?xml.*?\?>', '', robot_desc).strip()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_desc
        }]
    )

    # =========================================================================
    # 4. GAZEBO SIMULATION
    # =========================================================================

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

    spawn_entity_node = Node(
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

    # Slight delay to ensure Gazebo is ready before spawning
    spawn_entity_delayed = TimerAction(
        period=2.0,
        actions=[spawn_entity_node]
    )

    # =========================================================================
    # 5. ROS 2 CONTROLLERS
    # =========================================================================

    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager', '--controller-manager-timeout', '120'],
        output='screen'
    )

    gripper_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '-c', '/controller_manager', '--controller-manager-timeout', '120'],
        output='screen'
    )

    # Launch controllers only after the robot is successfully spawned
    spawn_controllers_event = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=[jsb_spawner, gripper_spawner]
        )
    )

    # =========================================================================
    # 6. NAVIGATION & EXPLORATION
    # =========================================================================

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup, 'launch', 'slam_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
            'transform_timeout': '0.5'
        }.items()
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
            'log_level': 'Fatal'
        }.items()
    )

    explore_node = Node(
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
    )

    # Delay exploration start to let SLAM stabilize
    explore_delayed = TimerAction(period=12.0, actions=[explore_node])

    # =========================================================================
    # 7. VISION & AI
    # =========================================================================

    stereo_proc_group = GroupAction(
        actions=[
            PushRosNamespace('oakd'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_stereo_image_proc, 'launch', 'stereo_image_proc.launch.py')),
                launch_arguments={
                    'approximate_sync': 'True',
                    'use_sim_time': 'True',
                    'left_namespace': 'left',
                    'right_namespace': 'right',
                }.items()
            )
        ]
    )

    ai_node = Node(
        package='tb3_autonomy',
        executable='sim_yolo_depth',
        name='ai_node',
        output='screen',
        parameters=[{
            'weights': yolo_model_path,
            'device': 'cpu',
            'conf': 0.5,
            'debug_view': True
        }]
    )

    # =========================================================================
    # 8. SUPERVISION & TOOLS
    # =========================================================================

    catch_node = Node(
        package='tb3_autonomy',
        executable='catch_node',
        name='catch_node',
        output='screen',
        parameters=[{'use_sim_time': True, 'mode': 'sim'}],
    )

    bt_supervisor_node = Node(
        package='tb3_autonomy',
        executable='bt_supervisor',
        name='bt_supervisor',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': True}],
        output='log'
    )

    # Start High-Level Logic only after controllers are ready (Gripper needs to be active)
    start_logic_event = RegisterEventHandler(
        OnProcessExit(
            target_action=gripper_spawner,
            on_exit=[bt_supervisor_node, catch_node, ai_node, explore_delayed]
        )
    )

    # =========================================================================
    # 9. FINAL LAUNCH DESCRIPTION
    # =========================================================================

    ld = LaunchDescription()

    # Environment
    for var in set_env_vars:
        ld.add_action(var)

    # Simulation & Robot
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(spawn_entity_delayed)
    ld.add_action(spawn_controllers_event)

    # Navigation
    ld.add_action(slam_launch)
    ld.add_action(navigation_launch)

    # Vision & Tools
    ld.add_action(rviz_node)
    ld.add_action(stereo_proc_group)

    # Logic (Delayed)
    ld.add_action(start_logic_event)

    return ld