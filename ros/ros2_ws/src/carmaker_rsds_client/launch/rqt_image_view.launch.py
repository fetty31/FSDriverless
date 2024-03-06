from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():

     rqt_image_view_node = Node(
          namespace=LaunchConfiguration('camera_name'),
          package='rqt_image_view',
          executable='rqt_image_view',
          name='rqt_image_view',
          output='screen',
          arguments=[PathJoinSubstitution(['/', LaunchConfiguration("camera_name"), 'image_raw'])],
          condition=IfCondition(LaunchConfiguration("rqt_image_view")))

     return LaunchDescription([
          rqt_image_view_node
     ])
