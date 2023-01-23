from launch import LaunchDescription
import launch_ros.actions
import os

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            namespace= "mpc_solver", package='px4_ros_com/', executable='LinearModelPredictiveControl', output='screen'),
        launch_ros.actions.Node(
            namespace= "joystick_input", package='px4_ros_com', executable='turtlesim_node', output='screen'),
    ])