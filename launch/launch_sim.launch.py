import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os



def generate_launch_description():
    # Package and directory paths
    package_name = 'my_bot'
    package_share_directory = get_package_share_directory(package_name)

    # ✅ Robot State Publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            package_share_directory, 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # ✅ Gazebo world path
    world = os.path.join(package_share_directory, 'worlds', 'empty.world')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
        )]),
        launch_arguments={
            'world': world,
            'gui': 'true',             
            'verbose': 'flase'
        }.items()
    )


    # ✅ Spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_bot',
            '-x', '1.0',
            '-y', '0.0',
            '-z', '0.0',
            '-Y', '1.5708'
        ],
        output='log'  # Directing output to log
    )

    # ✅ RViz2 configuration file path
    rviz_config_file = os.path.join(package_share_directory, 'config', 'view_bot.rviz')

    # ✅ Launch RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',  # Directing output to log
        arguments=['-d', rviz_config_file],
    )


    # ✅ Additional node
    base_link_coordinates_node = Node(
        package=package_name,
        executable='tf_test.py',
        name='base_coordinates',
        output='log',  # Directing output to log
        emulate_tty=True
    )
    
    # ✅ Path Smoother Node (with logging)
    path_smoother_node = Node(
        package=package_name,
        executable='path.py',  # Your path smoother script
        name='path_smoother',
        output='screen',  # Show output in terminal
        emulate_tty=True
    )
    
    # ✅ Path Executor Node (with logging)
    waypoints = Node(
        package=package_name,
        executable='waypoint.py',  # Your path executor script
        name='path_executor',
        output='screen',  # Show output in terminal
        emulate_tty=True
    )

    # Directly control logging for all nodes
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        rviz_node,
        base_link_coordinates_node,
        path_smoother_node,      # ✅ Path Smoother with logging
        waypoints,       # ✅ Path Executor with logging
    ])
