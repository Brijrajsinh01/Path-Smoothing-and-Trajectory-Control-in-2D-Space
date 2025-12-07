import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled

    package_name='my_bot' 
    package_share_directory = get_package_share_directory(package_name)
    
    return LaunchDescription([
            Node(
                            package=package_name,
                            executable='navigation.py',
                            name='my_script_node',
                            output='screen',
                            prefix=['python3'],
                            arguments=[os.path.join(package_share_directory, 'scripts', 'navigation.py')],
                    )
    ])