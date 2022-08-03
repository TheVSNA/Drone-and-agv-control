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

    world = '/home/drone/PX4-Autopilot/Tools/sitl_gazebo/worlds/empty.world'
    model1 = os.path.abspath(os.path.join(os.getcwd(),'..','models','iris1.sdf'))
    model2 = os.path.abspath(os.path.join(os.getcwd(),'..','models','iris2.sdf'))

    os.makedirs(PX4_RUN_DIR, exist_ok=True)

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH',
                            HOME + '/PX4-Autopilot/build/px4_sitl_rtps/build_gazebo'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', HOME + '/PX4-Autopilot/Tools/sitl_gazebo/models'),

        SetEnvironmentVariable('PX4_SIM_MODEL', 'iris'),

        DeclareLaunchArgument('model', default_value=model1),
        DeclareLaunchArgument('x', default_value='-700.5'), 
        DeclareLaunchArgument('y', default_value='704.0'),  
        DeclareLaunchArgument('z', default_value='472.0'),
        DeclareLaunchArgument('R', default_value='0.0'),
        DeclareLaunchArgument('P', default_value='0.0'),
        DeclareLaunchArgument('Y', default_value='0.0'),


        ExecuteProcess( #spawn first drone
            cmd=[
                'gz', 'model',
                '--spawn-file', LaunchConfiguration('model'),
                '--model-name', 'drone1',
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
                HOME + '/PX4-Autopilot/build/px4_sitl_rtps/bin/px4', '-i ','1',
                HOME + '/PX4-Autopilot/ROMFS/px4fmu_common/',
                '-s',
                HOME + '/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/rcS',
            ],
            cwd=PX4_RUN_DIR,
            output='screen'),

        ExecuteProcess( #start px4-ros2 bridge
            cmd=['micrortps_agent', '-t', 'UDP','-r','2022','-s','2021','-n','iris0'],
            output='screen'),

        DeclareLaunchArgument('model2', default_value=model2),
        DeclareLaunchArgument('x2', default_value='-700.15'), 
        DeclareLaunchArgument('y2', default_value='706.0'),  
        DeclareLaunchArgument('z2', default_value='472.0'),
        DeclareLaunchArgument('R2', default_value='0.0'),
        DeclareLaunchArgument('P2', default_value='0.0'),
        DeclareLaunchArgument('Y2', default_value='0.0'),

        ExecuteProcess( #spawn second drone
            cmd=[
                'gz', 'model',
                '--spawn-file', LaunchConfiguration('model2'),
                '--model-name', 'drone2',
                '-x', LaunchConfiguration('x2'),
                '-y', LaunchConfiguration('y2'),
                '-z', LaunchConfiguration('z2'),
                '-R', LaunchConfiguration('R2'),
                '-P', LaunchConfiguration('P2'),
                '-Y', LaunchConfiguration('Y2')
            ],
            prefix="bash -c 'sleep 5s; $0 $@'",
            output='screen'),

        ExecuteProcess( 
            cmd=[
                HOME + '/PX4-Autopilot/build/px4_sitl_rtps/bin/px4', '-i','2',
                HOME + '/PX4-Autopilot/ROMFS/px4fmu_common/',
                '-s',
                HOME + '/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/rcS',
            ],
            cwd=PX4_RUN_DIR,
            output='screen'),

        ExecuteProcess( #start px4-ros2 bridge
            cmd=['micrortps_agent', '-t', 'UDP','-r','2024','-s','2023','-n','iris1'],
            output='screen'),
        
])