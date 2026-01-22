import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file pour le turtlebot
    """

    # --- Configuration ---
    default_blob = 'tb3_autonomy/best_openvino_2022.1_6shave.blob'

    blob_path_arg = DeclareLaunchArgument(
        'blob_path',
        default_value=default_blob,
        description='Chemin absolu vers le modèle BLOB MyriadX'
    )

    # --- Node Vision (OAK-D) ---
    oak_node = Node(
        package='tb3_autonomy',
        executable='oak_yolo_depth',
        name='oak_yolo_depth',
        output='screen',
        parameters=[{
            'blob_path': LaunchConfiguration('blob_path'),
            'conf_thres': 0.5,
            'debug_view': False,
            'camera_frame': 'oak_d_pro_color_optical_frame'
        }]
    )

    # --- Node Pince (Mode Réel) ---
    catch_node = Node(
        package='tb3_autonomy',
        executable='catch_node',
        name='catch_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'mode': 'real'
        }]
    )

    servo_driver = Node(
        package='tb3_autonomy',
        executable='servo_driver',
        name='servo_driver',
        output='screen'
    )

    return LaunchDescription([
        blob_path_arg,
        oak_node,
        catch_node,
        servo_driver
    ])