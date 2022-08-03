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
    #model1 = os.path.abspath(os.path.join(os.getcwd(),'..','models','iris1.sdf'))
    #model2 = os.path.abspath(os.path.join(os.getcwd(),'..','models','iris2.sdf'))
    model3 = os.path.abspath(os.path.join(os.getcwd(),'..','models','iris3.sdf'))

    #model1 = '/home/drone/PX4-Autopilot/Tools/sitl_gazebo/models/iris/iris1.sdf'
    #model2 = '/home/drone/PX4-Autopilot/Tools/sitl_gazebo/models/iris/iris2.sdf'
    os.makedirs(PX4_RUN_DIR, exist_ok=True)

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH',
                            HOME + '/PX4-Autopilot/build/px4_sitl_rtps/build_gazebo'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', HOME + '/PX4-Autopilot/Tools/sitl_gazebo/models'),

        SetEnvironmentVariable('PX4_SIM_MODEL', 'iris'),

        DeclareLaunchArgument('model3', default_value=model3),
        DeclareLaunchArgument('x3', default_value='-0.15'), 
        DeclareLaunchArgument('y3', default_value='-3.0'),  
        DeclareLaunchArgument('z3', default_value='2.0'),
        DeclareLaunchArgument('R3', default_value='0.0'),
        DeclareLaunchArgument('P3', default_value='0.0'),
        DeclareLaunchArgument('Y3', default_value='0.0'),

        ExecuteProcess(
            cmd=[
                'gz', 'model',
                '--spawn-file', LaunchConfiguration('model3'),
                '--model-name', 'drone3',
                '-x', LaunchConfiguration('x3'),
                '-y', LaunchConfiguration('y3'),
                '-z', LaunchConfiguration('z3'),
                '-R', LaunchConfiguration('R3'),
                '-P', LaunchConfiguration('P3'),
                '-Y', LaunchConfiguration('Y3')
            ],
            prefix="bash -c 'sleep 5s; $0 $@'",
            output='screen'),

            ExecuteProcess( 
            cmd=[
                HOME + '/PX4-Autopilot/build/px4_sitl_rtps/bin/px4', '-i','3',
                HOME + '/PX4-Autopilot/ROMFS/px4fmu_common/',
                '-s',
                HOME + '/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/rcS',
            ],
            cwd=PX4_RUN_DIR,
            output='screen'),

            ExecuteProcess( #start px4-ros2 bridge
            cmd=['micrortps_agent', '-t', 'UDP','-r','2026','-s','2025','-n','iris2'],
            output='screen')
        
])