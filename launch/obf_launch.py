from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory  
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    launch_file = LaunchConfiguration("launch_file", default=get_package_share_directory("rplidar_ros") + "/launch/rplidar_a2m8_launch.py")
    return LaunchDescription([
        # Launch RPLidar
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(launch_file),
        ),
        
        # Launch controler node
        Node(
            package='controller_server',
            executable='controller_server',
            name='controller_node',
            output='screen'
        ),

        # Launch servo node
        Node(
            package='servo_client',
            executable='servo_client',
            name='servo_node',
            output='screen'
        ),


    ])
