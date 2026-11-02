import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    package_dir = get_package_share_directory('assembly_line')

    robot_xacro = os.path.join(
        package_dir,
        'description',
        'robot',
        'industrial_robot.urdf.xacro'
    )

    robot_description = ParameterValue(
        Command(['xacro ', robot_xacro]),
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='industrial_robot',
        parameters=[
            {
                'robot_description': robot_description
            }
        ],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher
    ])
