import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, GroupAction, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    urdf_file_name = 'turtlebot3_ultimate.urdf'
    use_sim_time = 'true'

    pkg_tb3_autonomy = get_package_share_directory('tb3_autonomy')
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_stereo_image_proc = get_package_share_directory('stereo_image_proc')
    pkg_tb3_description = get_package_share_directory('turtlebot3_description')


    gazebo_model_path = os.path.join(pkg_tb3_description, '..') + ':' + os.path.join(pkg_tb3_autonomy, '..')

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=gazebo_model_path
    )

    env_lidar = SetEnvironmentVariable(name='LDS_MODEL', value='LDS-01')
    env_gl = SetEnvironmentVariable(name='LIBGL_ALWAYS_SOFTWARE', value='0')

    urdf_path = os.path.join(pkg_tb3_autonomy, 'urdf', urdf_file_name)

    doc = xacro.process_file(urdf_path)
    robot_desc = doc.toxml()

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
            '-z', '0.15'
        ],
        output='screen'
    )

    world_path = os.path.join(pkg_tb3_gazebo, 'worlds', 'turtlebot3_world.world')

    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    #nav2_params = os.path.join(pkg_nav2_bringup, 'params', 'nav2_params.yaml')
    pkg_tb3_autonomy = get_package_share_directory('tb3_autonomy')
    
    nav2_params = os.path.join(pkg_tb3_autonomy, 'params', 'my_nav2_params.yaml')
    slam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'slam_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
            'transform_timeout': '0.5'
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
            'planner_frequency': 0.33,
            'progress_timeout': 30.0,
            'potential_scale': 3.0,
            'orientation_scale': 0.0,
            'gain_scale': 1.0,
            'transform_tolerance': 0.3,
            'min_frontier_size': 0.75,
        }]
    )

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
        output='screen'
    )

    supervisor = Node(
        package='tb3_autonomy',
        executable='supervisor_node',
        name='supervisor_node',
        output='screen'
    )

    return LaunchDescription([
        set_gazebo_model_path,
        env_lidar,
        env_gl,
        gazebo_cmd,
        robot_state_publisher_cmd,
        spawn_entity_cmd,
        slam_cmd,
        navigation_cmd,
        explore_cmd,
        rviz_cmd,
        supervisor,
        stereo_proc
    ])