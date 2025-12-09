echo "=== Starting build process ==="
cd ~/ros2_ws
colcon build --symlink-install
echo "=== Build completed ==="

echo "=== Sourcing environment ==="
source install/setup.bash
echo "=== Environment sourced ==="

echo "=== Killing existing processes ==="
pkill -f gz
pkill -f ruby
pkill -f sim_server
pkill -f parameter_bridge
pkill -f rviz2
pkill -f robot_state_publisher
pkill -f ros2
echo "=== Processes cleaned ==="

echo "=== Setting up workspace and launching auto_explore ==="
cd ~/ros2_ws
source install/setup.bash
ros2 launch tb3_autonomy auto_explore.launch.py

echo "=== Launching object_detector ==="
cd ~/ros2_ws
source install/setup.bash
ros2 run tb3_autonomy object_detector