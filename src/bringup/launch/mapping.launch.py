import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

ld=LaunchDescription()

serial_py_launch=IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory("serial_py"),"launch","serial_py.launch.py")
        )
)

data_process_launch=IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
    	os.path.join(get_package_share_directory("data_process"),"launch","data_process.launch.py")
    )
)

laser_serial_port=LaunchConfiguration('serial_port', default='/dev/ttyUSB_laser')
laser_frame_id=LaunchConfiguration('frame_id', default='base_footprint')
laser_launch=IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
    	os.path.join(get_package_share_directory("sllidar_ros2"),"launch","sllidar_a3_launch.py")
    ),
    launch_arguments={'serial_port': laser_serial_port,
                      'frame_id':laser_frame_id,
                      }.items(),
)

cartographer_configuration=LaunchConfiguration('configuration_basename', default='fishbot_laser_2d.lua')
use_sim_time = LaunchConfiguration('use_sim_time', default='false')
cartographer_launch=IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
    	os.path.join(get_package_share_directory("fishbot_cartographer"),"launch","cartographer.launch.py")
    ),
    launch_arguments={'configuration_basename': cartographer_configuration,
                      'use_sim_time':use_sim_time,
                      }.items(),
)

display_module=LaunchConfiguration("display_module", default="false")
fake_joint_pub=LaunchConfiguration("fake_test", default="false")
description_launch=IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
    	os.path.join(get_package_share_directory("chassis"),"launch","display.launch.py")
    ),
    launch_arguments={'display_module':display_module,
                      'fake_test':fake_joint_pub,
                      }.items(),
)

#ld.add_action(serial_py_launch)
#ld.add_action(data_process_launch)
ld.add_action(laser_launch)
ld.add_action(cartographer_launch)
#ld.add_action(description_launch)

def generate_launch_description():
    return ld
