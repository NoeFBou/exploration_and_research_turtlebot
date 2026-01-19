import os
from glob import glob

from setuptools import setup

package_name = "tb3_autonomy"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name, package_name + '.behaviors', package_name + '.nodes'],    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*.sdf")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.urdf")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.xacro")),

        (os.path.join("share", package_name, "params"), glob("params/*.yaml")),
        (os.path.join("share", package_name, "models"), glob("models/*.pt")),
        (os.path.join("share", package_name, "meshes", "bases"), glob("meshes/bases/*.stl")),
        (os.path.join("share", package_name, "meshes", "wheels"), glob("meshes/wheels/*.stl")),
        (os.path.join("share", package_name, "meshes", "sensors"), glob("meshes/sensors/*.stl")),
        (os.path.join("share", package_name, "meshes", "sensors"), glob("meshes/sensors/*.dae")),
        (os.path.join("share", package_name, "meshes", "sensors"), glob("meshes/sensors/*.png")),
        (os.path.join("share", package_name, "meshes", "actuators", "gripper"), glob("meshes/actuators/gripper/*.stl")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="nono",
    maintainer_email="nono@todo.todo",
    description="Projet exploration Turtlebot3",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "supervisor = tb3_autonomy.mission_supervisor:main", # Si celui-ci n'a pas bougé, ne pas toucher
            "bt_supervisor = tb3_autonomy.bt_supervisor:main",   # Celui-ci est à la racine de tb3_autonomy/
            "mission_controller = tb3_autonomy.nodes.controller:main",
            "catch_node = tb3_autonomy.nodes.catch_node:main",
            "sim_yolo_depth = tb3_autonomy.nodes.sim_yolo_depth_node:main",
            "oak_yolo_depth = tb3_autonomy.nodes.oak_yolo_depth_node:main",
        ],
    },
    # entry_points={
    #     "console_scripts": [
    #         "supervisor = tb3_autonomy.mission_supervisor:main",
    #         "supervisor_node = tb3_autonomy.supervisor_node:main",
    #         "camera_ai = tb3_autonomy.camera_processor:main",
    #         "object_detector = tb3_autonomy.object_detector:main",
    #         "catch_node = tb3_autonomy.catch_node:main",
    #         "sim_yolo_depth = tb3_autonomy.sim_yolo_depth_node:main",
    #         "oak_yolo_depth = tb3_autonomy.oak_yolo_depth_node:main",
    #         'bt_supervisor = tb3_autonomy.bt_supervisor:main',
    #     ],
    # },
)
