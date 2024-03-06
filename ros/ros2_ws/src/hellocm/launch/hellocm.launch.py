from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'ns',
            default_value='/hellocm',
            description='Node namespace'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use node with simulation time'),
        DeclareLaunchArgument(
            'cycletime',
            default_value='10000',
            description='Node cycle time'),
        Node(
            namespace=LaunchConfiguration('ns'),
            package="hellocm",
            executable='hellocm',
            name='hellocm',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'cycletime': LaunchConfiguration('cycletime')},
            ],
            remappings=[
              ([LaunchConfiguration('ns'), '/cm2ext'], '/carmaker/cm2ext'),
            ]
        )
    ])
