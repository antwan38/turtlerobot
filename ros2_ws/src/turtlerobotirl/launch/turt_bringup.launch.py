import launch
import launch_ros.actions
from launch_ros.actions import Node

def generate_launch_description():
    urdf_file = "/home/antwan/ros2_ws/src/turtlerobotirl/urdf/nox.urdf"
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return launch.LaunchDescription([
        Node(
            package='turtlerobotirl',
            executable='odomTranslator',
            name='odomTranslator',
            parameters=[{'use_sim_time': False}]),
        Node(
            package='turtlerobotirl',
            executable='arduinoTranslator',
            name='arduinoTranslator',
            parameters=[{'use_sim_time': False}]),
        Node(
            name='rplidar_composition',
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,  # A1 / A2
                # 'serial_baudrate': 256000, # A3
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
                'use_sim_time': False
            }],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='both',
            parameters=[{'robot_description': robot_desc,
    			'use_sim_time': False}]),
  ])
