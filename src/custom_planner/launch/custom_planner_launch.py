from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare the launch arguments
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Top-level namespace'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'params_file',
            default_value='/home/justin/navigation2/nav2_bringup/params/nav2_params.yaml',
            description=''),

        # Include the default nav2_bringup launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                LaunchConfiguration('install_dir', default='/opt/ros/humble'),
                '/share/nav2_bringup/launch/navigation_launch.py'
            ]),
            launch_arguments={
                'namespace': LaunchConfiguration('namespace'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': LaunchConfiguration('params_file')
            }.items()
        ),

        # Launch the custom_planner node
        Node(
            package='custom_planner',
            executable='straight_line_planner',
            name='straight_line_planner',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        )
    ])


