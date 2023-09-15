import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction

ld=LaunchDescription()
map_pkg=get_package_share_directory('bringup')
map_name='map_104.yaml'
map_yaml_path=LaunchConfiguration('map',default=os.path.join(map_pkg,"map",map_name))
map_keepout_name='map_104_keepout.yaml'
mask_yaml_file=LaunchConfiguration('mask',default=os.path.join(map_pkg,"map",map_keepout_name))
# map_name='rmuc2023.yaml'
# map_keepout_name='rmuc2023_keepout.yaml'
use_sim_time = LaunchConfiguration('use_sim_time', default='false')

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

nav_ctrl_launch=IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
    	os.path.join(get_package_share_directory("nav_ctrl"),"launch","nav_ctrl.launch.py")
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

nav2_param_path=LaunchConfiguration('params_file',default=os.path.join(get_package_share_directory("fishbot_navigation2"),"param","fishbot.yaml"))
nav2_bringup_launch=IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
    	os.path.join(get_package_share_directory("nav2_bringup"),"launch","bringup_launch.py")
    ),
    launch_arguments={'map': map_yaml_path,
                      'use_sim_time':use_sim_time,
                      'params_files':nav2_param_path,
                      }.items(),
)

ekf_launch=IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
    	os.path.join(get_package_share_directory("fishbot_navigation2"),"launch","odom_ekf.launch.py")
    ),
    launch_arguments={'use_sim_time':use_sim_time,
                      }.items(),
)

tf_broadcaster=IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory("tf_broadcaster"),"launch","tf_broadcaster.launch.py")
        )
)

tf_static=Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    arguments=['0','0','0','0','0','0','base_footprint','base_link']
)

configuration_directory = LaunchConfiguration('configuration_directory',default= os.path.join(get_package_share_directory("fishbot_cartographer"),'config'))
configuration_basename = LaunchConfiguration('configuration_basename', default='fishbot_laser_2d.lua')
cartographer_node = Node(
    package='cartographer_ros',
    executable='cartographer_node',
    name='cartographer_node',
    #output='screen',
    parameters=[{'use_sim_time': use_sim_time}],
    arguments=['-configuration_directory', configuration_directory,
               '-configuration_basename', configuration_basename]
)

costmap_filter_param=LaunchConfiguration('params_file',default=os.path.join(get_package_share_directory("costmap_filters"),"params","keepout_params.yaml"))
costmap_filter_launch=IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
    	os.path.join(get_package_share_directory("costmap_filters"),"launch","costmap_filter.launch.py")
    ),
    launch_arguments={'use_sim_time':use_sim_time,
                      'params_file':costmap_filter_param,
                      'mask':mask_yaml_file,
                      }.items(),
)
costmap_filter_delay_launch=TimerAction(period=3.0,actions=[costmap_filter_launch])

rviz_config_dir=os.path.join(get_package_share_directory("nav2_bringup"),"rviz","nav2_default_view.rviz")
rviz_node=Node(
    package="rviz2",
    executable="rviz2",
    name="rviz2",
    arguments=['-d',rviz_config_dir],
    parameters=[{'use_sim_time':use_sim_time}],
    #output='screen'
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

ld.add_action(serial_py_launch)
ld.add_action(data_process_launch)
ld.add_action(nav_ctrl_launch)
ld.add_action(laser_launch)
ld.add_action(nav2_bringup_launch)
#ld.add_action(ekf_launch)
#ld.add_action(tf_broadcaster)
ld.add_action(tf_static)
ld.add_action(cartographer_node)
ld.add_action(costmap_filter_delay_launch)
ld.add_action(rviz_node)
#ld.add_action(description_launch)

def generate_launch_description():
    return ld
