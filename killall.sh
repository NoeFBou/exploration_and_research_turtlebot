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