#!/usr/bin/env python3

from multiprocessing.dummy import Process
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
    PX4_RUN_DIR = HOME + '/tmp' #'/tmp/px4_run_dir'
    gazebo_launch_dir = os.path.join(get_package_share_directory('gazebo_ros'), 'launch')

    world = '/home/drone/PX4-Autopilot/Tools/sitl_gazebo/worlds/myworld.world'
    model4 = os.path.abspath(os.path.join(os.getcwd(),'..','models','testrover1.sdf'))
    os.makedirs(PX4_RUN_DIR, exist_ok=True)

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH',
                            HOME + '/PX4-Autopilot/build/px4_sitl_rtps/build_gazebo'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', HOME + '/PX4-Autopilot/Tools/sitl_gazebo/models'),

        SetEnvironmentVariable('PX4_SIM_MODEL', 'rover'),

        DeclareLaunchArgument('world', default_value=world),
        DeclareLaunchArgument('model4', default_value=model4),
        DeclareLaunchArgument('x4', default_value='-700.0'),
        DeclareLaunchArgument('y4', default_value='700.0'),
        DeclareLaunchArgument('z4', default_value='471.5'),
        DeclareLaunchArgument('R4', default_value='0.0'),
        DeclareLaunchArgument('P4', default_value='0.0'),
        DeclareLaunchArgument('Y4', default_value='-1.57'),

        IncludeLaunchDescription(   #start gazebo "server" side
            PythonLaunchDescriptionSource([gazebo_launch_dir, '/gzserver.launch.py']),
            launch_arguments={'world': LaunchConfiguration('world'),
                            'verbose': 'true'}.items(),
        ),
        IncludeLaunchDescription(   #start gazebo "client" side
            PythonLaunchDescriptionSource([gazebo_launch_dir, '/gzclient.launch.py'])
        ),

        ExecuteProcess( #spawn first rover
            cmd=[
                'gz', 'model',
                '--spawn-file', LaunchConfiguration('model4'),
                '--model-name', 'rover1',
                '-x', LaunchConfiguration('x4'),
                '-y', LaunchConfiguration('y4'),
                '-z', LaunchConfiguration('z4'),
                '-R', LaunchConfiguration('R4'),
                '-P', LaunchConfiguration('P4'),
                '-Y', LaunchConfiguration('Y4')
            ],
            prefix="bash -c 'sleep 5s; $0 $@'",
            output='screen'),

        ExecuteProcess(
            cmd=[
                HOME + '/PX4-Autopilot/build/px4_sitl_rtps/bin/px4', '-i','0',
                HOME + '/PX4-Autopilot/ROMFS/px4fmu_common/',
                '-s',
                HOME + '/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/rcS',
            ],
            cwd=PX4_RUN_DIR,
            output='screen'),

        ExecuteProcess( #start px4-ros2 bridge
            cmd=['micrortps_agent', '-t', 'UDP','-r','2020','-s','2019','-n','rover0'],
            output='screen'),
])