echo "=== Killing processes ==="
pkill -f ros2
pkill -f gz
pkill -f ruby
pkill -f sim_server
pkill -f parameter_bridge
pkill -f rviz2
pkill -9 gzserver
pkill -f gzclient
pkill -f robot_state_publisher
killall -9 _ros2_daemon 2>/dev/null
echo "=== Processes killed ==="

echo "=== Restarting ROS2 daemon ==="
ros2 daemon stop 2>/dev/null
ros2 daemon start
echo "=== ROS2 daemon restarted ==="

echo "=== Cleaning cached robot models from world file ==="
sed -i '/    <model name=.my_custom_waffle.>/,/    <\/model>/d' worlds/my_room.sdf
echo "=== World file cleaned ==="

echo "=== Starting build process ==="
cd ../../
colcon build --symlink-install
echo "=== Build completed ==="

echo "=== Sourcing environment ==="
source install/setup.bash
echo "=== Environment sourced ==="

sleep 4
echo "=== Setting up workspace and launching auto_explore ==="
source install/setup.bash
ros2 launch tb3_autonomy auto_explore.launch.py


pkill -f ros2
pkill -f gz
pkill -f ruby
pkill -f sim_server
pkill -f parameter_bridge
pkill -f rviz2
pkill -9 gzserver
pkill -f gzclient
pkill -f robot_state_publisher
killall -9 _ros2_daemon 2>/dev/null
ros2 daemon stop 2>/dev/null
ros2 daemon start
ros2 launch tb3_autonomy auto_explore.launch.py