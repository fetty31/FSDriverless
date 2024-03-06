import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import (EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.substitutions import LaunchConfiguration, LocalSubstitution, EnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.events import Shutdown

def generate_launch_description():

     # Arguments: IPG Movie
     carmaker_host_arg = DeclareLaunchArgument(
          'carmaker_host',
          default_value='localhost',
          description='Host on which CarMaker executable is running')  

     # Arguments: RSDS Client
     camera_no_arg = DeclareLaunchArgument(
          'camera_no',
          default_value='0',
          description='Camera number')

     rsds_host_arg = DeclareLaunchArgument(
          'rsds_host',
          default_value=LaunchConfiguration("carmaker_host"),
          description='Host on which IPGMovie instance is running')

     rsds_port_arg = DeclareLaunchArgument(
          'rsds_port',
          default_value='2210',
          description='Connection port defined in RSDS.conf file')

     connection_tries_arg = DeclareLaunchArgument(
          'connection_tries',
          default_value='5',
          description='How often RSDS client should try to connect to IPGMovie host')

     camera_name_arg = DeclareLaunchArgument(
          'camera_name',
          default_value='rsds_camera',
          description='Camera name for ROS topic')

     camera_frame_arg = DeclareLaunchArgument(
          'camera_frame',
          default_value='Fr1A',
          description='Name of camera frame_id for ROS topic')

     # Arguments: Start RQT
     rqt_image_view_arg = DeclareLaunchArgument(
        'rqt_image_view',
        default_value='True',
        description='Launch "rqt_image_view"')
     
     # Include Rviz2 node if required
     rqt_image_view_include = IncludeLaunchDescription(
          PythonLaunchDescriptionSource(
               os.path.join(
                    get_package_share_directory('carmaker_rsds_client'),
                    'launch/rqt_image_view.launch.py'
               )
          ),
          launch_arguments = {'rqt_image_view': LaunchConfiguration('rqt_image_view'),
                              'camera_name': LaunchConfiguration('camera_name')}.items()
     )

     # Arguments: External Coordinate Transformation
     param_trans_rot_arg = DeclareLaunchArgument(
          'param_trans_rot',
          description='Translation (in m) and rotation (Euler ZYX in degree) in Fr1A frame: x, y, z, roll, pitch, yaw')

     # Arguments: Camera Calibration (Default values based on distortion-free direct lens)
     fov_deg_arg = DeclareLaunchArgument(
          'fov_deg',
          default_value='60',
          description='Field of View in degree')

     width_arg = DeclareLaunchArgument(
          'width',
          default_value='1080',
          description='Image width')
          
     height_arg = DeclareLaunchArgument(
          'height',
          default_value='1920',
          description='Image height')
          
     c_x_arg = DeclareLaunchArgument(
          'c_x',
          default_value='540.0',
          description='Horizontal principal point')
          
     c_y_arg = DeclareLaunchArgument(
          'c_y',
          default_value='960.0',
          description='Vertical principal point')
          
     fov_rad_arg = DeclareLaunchArgument(
          'fov_rad',
          default_value='1.047',
          description='Field of View in radians')
          
     f_x_arg = DeclareLaunchArgument(
          'f_x',
          default_value='935.5',
          description='Horizontal focal length')
          
     f_y_arg = DeclareLaunchArgument(
          'f_y',
          default_value='1663.1',
          description='Vertical focal length')
          
     max_f_arg = DeclareLaunchArgument(
          'max_f',
          default_value='1663.1',
          description='Max. focal length, used for direct lense')

     calib_mat_k_arg = DeclareLaunchArgument(
          'calib_mat_k',
          default_value='[1663.1, 0.0, 540.0, 0.0, 1663.1, 960.0, 0.0, 0.0, 1.0]',
          description='Intrinsic camera matrix for the raw (distorted) images')

     calib_mat_r_arg = DeclareLaunchArgument(
          'calib_mat_r',
          default_value='[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]',
          description='Rectification matrix (stereo cameras only)')

     calib_mat_p_arg = DeclareLaunchArgument(
          'calib_mat_p',
          default_value='[1663.1, 0.0, 540.0, 0.0, 0.0, 1663.1, 960.0, 0.0, 0.0, 0.0, 1.0, 0.0]',
          description='Projection/camera matrix')

     calib_mat_d_arg = DeclareLaunchArgument(
          'calib_mat_d',
          default_value='[0.0]',
          description='The distortion parameters, size depending on the distortion model')

     distortion_model_arg = DeclareLaunchArgument(
          'distortion_model',
          default_value='plumb_bob',
          description='The distortion model used, leave empty if distortion-free. See sensor_msgs/distortion_models.h')

     binning_x_arg = DeclareLaunchArgument(
          'binning_x',
          default_value='0',
          description='Binning combines rectangular neighborhoods of pixels into larger pixels')

     binning_y_arg = DeclareLaunchArgument(
          'binning_y',
          default_value='0',
          description='Binning combines rectangular neighborhoods of pixels into larger pixels')

     # Launch the RSDS node
     rsds_node = Node(
          namespace=LaunchConfiguration('camera_name'),
          package="carmaker_rsds_client",
          executable='carmaker_rsds_client_node',
          name=LaunchConfiguration('camera_name'),
          output='screen',
          parameters=[
               {'camera_no': LaunchConfiguration('camera_no')},
               {'rsds_host': LaunchConfiguration('rsds_host')},
               {'rsds_port': LaunchConfiguration('rsds_port')},
               {'connection_tries': LaunchConfiguration('connection_tries')},
               {'camera_name': LaunchConfiguration('camera_name')},
               {'camera_frame': LaunchConfiguration('camera_frame')},
               {'param_trans_rot': LaunchConfiguration('param_trans_rot')},
               {'width': LaunchConfiguration('width')},
               {'height': LaunchConfiguration('height')},
               {'c_x': LaunchConfiguration('c_x')},
               {'c_y': LaunchConfiguration('c_y')},
               {'fov_rad': LaunchConfiguration('fov_rad')},
               {'f_x': LaunchConfiguration('f_x')},
               {'f_y': LaunchConfiguration('f_y')},
               {'max_f': LaunchConfiguration('max_f')},
               {'calib_mat_k': LaunchConfiguration('calib_mat_k')},
               {'calib_mat_r': LaunchConfiguration('calib_mat_r')},
               {'calib_mat_p': LaunchConfiguration('calib_mat_p')},
               {'calib_mat_d': LaunchConfiguration('calib_mat_d')},
               {'distortion_model': LaunchConfiguration('distortion_model')},
               {'binning_x': LaunchConfiguration('binning_x')},
               {'binning_y': LaunchConfiguration('binning_y')}
          ],
        prefix=["bash -c 'sleep 5; $0 $@' "]
    )

     process_exit_event = RegisterEventHandler(
          OnProcessExit(
               target_action=rsds_node,
               on_exit=[
                    LogInfo(msg=('CarMaker has shutdown this RSDS client node.')),
                    EmitEvent(event=Shutdown(
                        reason='SIGINT received'))
               ]
          )
     )

     return LaunchDescription([
          carmaker_host_arg,
          camera_no_arg,
          rsds_host_arg,
          rsds_port_arg,
          connection_tries_arg,
          camera_name_arg,
          camera_frame_arg,
          rqt_image_view_arg,
          param_trans_rot_arg,
          fov_deg_arg,
          width_arg,
          height_arg,
          c_x_arg,
          c_y_arg,
          fov_rad_arg,
          f_x_arg,
          f_y_arg,
          max_f_arg,
          calib_mat_k_arg,
          calib_mat_r_arg,
          calib_mat_p_arg,
          calib_mat_d_arg,
          distortion_model_arg,
          binning_x_arg,
          binning_y_arg,
          rqt_image_view_include,
          rsds_node,
          process_exit_event
    ])
