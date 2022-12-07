# Shared control MPC

This project is based on [PX4_Autopilot](https://github.com/PX4/PX4-Autopilot) and implements a MPC controller on a iris accepting user input via joystick on flight. To begin with this project, you have to ensure the enviromental dependencies have been installed:

1. ROS2 FOXY
2. PX4-ROS2 bridge
3. PX4-Aupilot(v1.13.1)

## Install PX4_ROS2 bridge

Check the [RTPS/ROS2 Interface](https://dev.px4.io/en/middleware/micrortps.html) section on the PX4 Devguide for details on how to install the required dependencies, build the package (composed by the two branches) and use it.

# Build and run
## build(first terminal)
```
source /opt/ros/foxy/setup.bash
cd path/to/px4_ros_com_ros2/
colcon build
```

## run gaezbo(second terminal)
```
cd path/to/PX4-Autopilot
make px4_sitl_rtps gazebo
```

## run micrortps_agent(third terminal)
```
source /opt/ros/foxy/setup.bash
micrortps_agent -t UDP
```

## run MPC controller node (forth terminal)
```
source /opt/ros/foxy/setup.bash
source /path/to/px4_ros_com_ros2/install/setup.bash
ros2 run px4_ros_com LinearPredictiveModel
```

## run tarjectory setpoint node (fifth terminal)
```
source /opt/ros/foxy/setup.bash
source /path/to/px4_ros_com_ros2/install/setup.bash
ros2 run px4_ros_com trajectory_pub
```
