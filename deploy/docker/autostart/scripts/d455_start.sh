sleep 2
WORKSPACE_DIR=/home/robocaster-orin/Documents/B1Py
source ${WORKSPACE_DIR}/deploy/ros2_ws/install/setup.bash && ros2 launch ${WORKSPACE_DIR}/deploy/docker/launch/hw_nodes/d455.launch.py 
