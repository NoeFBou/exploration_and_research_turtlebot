import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

def generate_launch_description():

    env_model = SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='waffle')
    env_lidar = SetEnvironmentVariable(name='LDS_MODEL', value='LDS-01')
    
    env_gl = SetEnvironmentVariable(name='LIBGL_ALWAYS_SOFTWARE', value='1')

    use_sim_time = 'true'
    
    # Chemins des packages standards
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    
    nav2_params = os.path.join(pkg_nav2_bringup, 'params', 'nav2_params.yaml')

    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_gazebo, 'launch', 'turtlebot3_world.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'x_pose': '-2.0',
            'y_pose': '-0.5'
        }.items()
    )

    # SLAM (Cartographie en temps réel)
    slam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'slam_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time, 
            'params_file': nav2_params
        }.items()
    )

    # Navigation
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

    
    # Explore Lite
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
    pkg_stereo_image_proc = get_package_share_directory('stereo_image_proc')

    stereo_proc = GroupAction(
        actions=[
            PushRosNamespace('oakd'), # Tout se passera dans /oakd/...
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_stereo_image_proc, 'launch', 'stereo_image_proc.launch.py')
                ),
                launch_arguments={
                    'approximate_sync': 'True', # Indispensable pour la simu
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
        env_model,
        env_lidar,
        env_gl,
        gazebo_cmd,
        slam_cmd,
        navigation_cmd,
        explore_cmd,
        rviz_cmd,
        supervisor,
        stereo_proc
    ])
