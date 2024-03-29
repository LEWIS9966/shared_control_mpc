#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    """Launch Gazebo with a drone running PX4 communicating over ROS 2."""
    HOME = os.environ.get('HOME')
    PX4_RUN_DIR = HOME + '/tmp/px4_run_dir'
    gazebo_launch_dir = os.path.join(get_package_share_directory('gazebo_ros'), 'launch')

    fpv_racing_gazebo_dir = HOME + '/PX4-Autopilot/Tools/sitl_gazebo'#get_package_share_directory('fpv_racing_gazebo')
    world = os.path.join(fpv_racing_gazebo_dir, 'worlds', 'empty.world') # _with_obstacle
    model = os.path.join(fpv_racing_gazebo_dir, 'models', 'iris_fpv_cam', 'iris_fpv_cam.sdf')
    #custom_gazebo_models = os.path.join(blackdrones_description_dir, 'models')
    #px4_init = os.path.join(blackdrones_description_dir, 'PX4-init')

    os.makedirs(PX4_RUN_DIR, exist_ok=True)

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH',
                               HOME + '/PX4-Autopilot/build/px4_sitl_rtps/build_gazebo'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', HOME + '/PX4-Autopilot/Tools/sitl_gazebo/models'),

        SetEnvironmentVariable('PX4_SIM_MODEL', 'iris'),

        DeclareLaunchArgument('world', default_value=world),
        DeclareLaunchArgument('model', default_value=model),
        # DeclareLaunchArgument('interactive', default_value="true"),
        # DeclareLaunchArgument('Y', default_value='0.0'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('R', default_value='0.0'),
        DeclareLaunchArgument('P', default_value='0.0'),
        DeclareLaunchArgument('Y', default_value='0.0'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([gazebo_launch_dir, '/gzserver.launch.py']),
            launch_arguments={'world': LaunchConfiguration('world'),
                              'verbose': 'true'}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([gazebo_launch_dir, '/gzclient.launch.py'])
        ),
        ExecuteProcess(
            cmd=[
                'gz', 'model',
                '--spawn-file', LaunchConfiguration('model'),
                '--model-name', 'drone',
                '-x', LaunchConfiguration('x'),
                '-y', LaunchConfiguration('y'),
                '-z', LaunchConfiguration('z'),
                '-R', LaunchConfiguration('R'),
                '-P', LaunchConfiguration('P'),
                '-Y', LaunchConfiguration('Y')
            ],
            prefix="bash -c 'sleep 5s; $0 $@'",
            output='screen'),

        ExecuteProcess(
            cmd=[
                HOME + '/PX4-Autopilot/build/px4_sitl_rtps/bin/px4',
                HOME + '/PX4-Autopilot/ROMFS/px4fmu_common/',
                '-s',
                HOME + '/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/rcS'
            ],
            cwd=PX4_RUN_DIR,
            output='screen'),
        #ExecuteProcess(
            #cmd=['micrortps_agent', '-t', 'UDP'],
            #output='screen'),
        # ExecuteProcess(
        #     cmd=['/home/kumo/QGroundControl.AppImage'],
        #     output='screen'),

])
