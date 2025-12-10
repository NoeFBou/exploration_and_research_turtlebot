echo "=== Starting build process ==="
cd ../../
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
source install/setup.bash
ros2 launch tb3_autonomy auto_explore.launch.py