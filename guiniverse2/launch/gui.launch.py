from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'sim_mode',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        Node(
            package='guiniverse2',
            executable='gui',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('sim_mode')}],
            remappings=[
                ('/clock', 'clock'), 
                ('/tf', 'tf'), 
                ('/tf_static', 'tf_static'), 
            ]
        )
    ])