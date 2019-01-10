# Robot_Manipulator_Manager
ROS2 Robot Manipulator Manager to control ROS1 Robot Manipulator Control Node. Work along side with `robot_manipulator_control` ros1 package. To enable ros1 and ros2 package to work together, ros12_bridge is needed.

Edit `../config/eef_pose.yaml` for pose input.

**Gripper Support**
- which also named as: eef (end effector)
- This Ros2 pkg supports both ros1 and ros2 gripper control

**TEDIOUS WORK IN PROGRESS....**

## Architecture Diagram

![alt text](/documentations/rmm_diagram.png?)


## Setup and Run code

**Shell A** : Ros2 robot manipulator manager
```
colcon build --symlink-install --packages-select robot_manipulator_manager
source ~/rmf/build/ros2/install/local_setup.bash
robot_manipulator_manager.py #python script`
ros2 run ros1_bridge dynamic_bridge
```

**Shell B** : Ros1 Ros2 Bridge
For detailed setup ros1 bridge in ros2, refer to [here](https://github.com/ros2/ros1_bridge).
```
. /opt/ros/melodic/setup.bash
# Or, source ~/catkin_ws/install_isolated/setup.bash
. <install-space-with-bridge>/setup.bash
export ROS_MASTER_URI=http://localhost:11311
ros2 run ros1_bridge dynamic_bridge
```

