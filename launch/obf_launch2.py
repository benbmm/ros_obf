from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory  
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess

def generate_launch_description():
    launch_file1 = LaunchConfiguration("launch_file1", default=get_package_share_directory("rplidar_ros") + "/launch/rplidar_a2m8_launch.py")
    launch_file2 = LaunchConfiguration("launch_file2", default=get_package_share_directory("mpu6050driver") + "/launch/mpu6050driver_launch.py")
    #launch_file3 = LaunchConfiguration("launch_file2", default=get_package_share_directory("imu_complementary_filter") + "/launch/complementary_filter.launch.py")
    launch_file3 = LaunchConfiguration("launch_file3", default=get_package_share_directory("imu_filter_madgwick") + "/launch/imu_filter.launch.py")
    return LaunchDescription([
        ExecuteProcess(
            cmd=['sh', '-c', 'gio trash /home/user/ros2_obf_ws/src/sensor_data/*'],
            output='log'
        ),
    
        # Execute servo_initial.py script
        ExecuteProcess(
            cmd=['python3', '/home/user/ros2_obf_ws/src/servo_initial.py'],
            output='log'
        ),
        
        # Launch mpu6050driver
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(launch_file2),
        ),
        # Launch imu_complementary_filter
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(launch_file3),
        ),

        # Launch RPLidar
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(launch_file1),
        ),

        # Launch adaptation node
        Node(
            package='adaptation_rl',
            executable='adaptation_rl',
            name='adaptation_node',
            output='log'
        ),
        
        # Launch controler node
        Node(
            package='controller_server',
            executable='controller_server',
            name='controller_node',
            output='log'
        ),

        # Launch servo node
        Node(
            package='servo_client',
            executable='servo_client',
            name='servo_node',
            output='log'
        ),
    ])
