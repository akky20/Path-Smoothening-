from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directories
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    pkg_dir = get_package_share_directory('trajectory_tracking')

    # Set Turtlebot3 model
    os.environ['TURTLEBOT3_MODEL'] = 'burger'

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(turtlebot3_gazebo_dir, 'launch', 'empty_world.launch.py')
        ])
    )

    # RViz launch
    rviz_config = os.path.join(pkg_dir, 'rviz', 'trajectory.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else []
    )

    # Trajectory tracking node
    trajectory_node = Node(
        package='trajectory_tracking',
        executable='trajectory_node',
        name='trajectory_tracking_node',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }]
    )

    # Delay trajectory node to let Gazebo start
    delayed_trajectory_node = TimerAction(
        period=5.0,
        actions=[trajectory_node]
    )

    # Your avoidance node
    avoidance_node = Node(
        package='trajectory_tracking',
        executable='obstacle_avoidance_node',
        name='obstacle_avoidance_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        gazebo,
        rviz,
        delayed_trajectory_node,
        avoidance_node
    ])
