import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    package_dir = get_package_share_directory('assembly_line')

    world_file = os.path.join(
        package_dir,
        'worlds',
        'factory.world'
    )

    models_dir = os.path.join(
        package_dir,
        'models'
    )
    
    plugin_dir = os.path.join(
        package_dir,
        'lib'
    )
    
    robot_controllers_file = os.path.join(
        package_dir,
        'config',
        'robot_controllers.yaml'
    )

    # -------------------------------------------------
    # CONVEYOR
    # -------------------------------------------------

    conveyor_xacro = os.path.join(
        package_dir,
        'description',
        'conveyor.urdf.xacro'
    )

    conveyor_description = ParameterValue(
        Command(['xacro ', conveyor_xacro]),
        value_type=str
    )

    # -------------------------------------------------
    # INDUSTRIAL ROBOT
    # -------------------------------------------------

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

    # -------------------------------------------------
    # DRILLING ROBOT (WORKSTATION 2)
    # -------------------------------------------------

    drilling_robot_xacro = os.path.join(
        package_dir,
        'description',
        'robot',
        'drilling_robot.urdf.xacro'
    )

    drilling_robot_description = ParameterValue(
        Command(['xacro ', drilling_robot_xacro]),
        value_type=str
    )
    
    # -------------------------------------------------
    # GAZEBO
    # -------------------------------------------------

    gazebo = ExecuteProcess(
        cmd=[
            'gazebo',
            '--verbose',
            world_file,
            '-s',
            'libgazebo_ros_factory.so'
        ],
        output='screen'
    )

    # -------------------------------------------------
    # CONVEYOR ROBOT STATE PUBLISHER
    # -------------------------------------------------

    conveyor_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {
                'robot_description': conveyor_description
            }
        ],
        output='screen'
    )

    # -------------------------------------------------
    # CONVEYOR SPAWN
    # -------------------------------------------------

    spawn_conveyor = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity',
            'conveyor',
            '-topic',
            'robot_description'
        ],
        output='screen'
    )

    # -------------------------------------------------
    # INDUSTRIAL ROBOT STATE PUBLISHER
    # -------------------------------------------------

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='workstation1',
        parameters=[
            {
                'robot_description': robot_description
            }
        ],
        output='screen'
    )

    # -------------------------------------------------
    # DRILLING ROBOT STATE PUBLISHER
    # -------------------------------------------------

    drilling_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='workstation2',
        parameters=[
            {
                'robot_description': drilling_robot_description,
                'frame_prefix': 'workstation2_'
            }
        ],
        output='screen'
    )
    
    # -------------------------------------------------
    # INDUSTRIAL ROBOT SPAWN
    # -------------------------------------------------

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity',
            'workstation1_robot',
            '-robot_namespace',
            'workstation1',
            '-topic',
            '/workstation1/robot_description'
        ],
        output='screen'
    )

    # -------------------------------------------------
    # DRILLING ROBOT SPAWN
    # -------------------------------------------------

    spawn_drilling_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity',
    'workstation2_robot',
    '-robot_namespace',
    'workstation2',
    '-topic',
    '/workstation2/robot_description'
        ],
        output='screen'
    )
    
    # -------------------------------------------------
    # WORKPIECE
    # -------------------------------------------------

    workpiece_sdf = os.path.join(
        package_dir,
        'models',
        'workpiece',
        'model.sdf'
    )

    spawn_workpiece = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity',
            'workpiece',
            '-file',
            workpiece_sdf,
            '-x',
            '-0.6',
            '-y',
            '0.0',
            '-z',
            '0.87'
        ],
        output='screen'
    )

    # -------------------------------------------------
    # WORKPIECE AUTOMATION NODES
    # -------------------------------------------------

    workpiece_controller_node = Node(
        package='assembly_line',
        executable='workpiece_controller',
        output='screen'
    )

    workpiece_sensor_node = Node(
        package='assembly_line',
        executable='workpiece_sensor',
        output='screen'
    )
    
    # -------------------------------------------------
    # ROBOT AUTOMATION NODES
    # -------------------------------------------------

    drilling_controller_node = Node(
        package='assembly_line',
        executable='drilling_controller',
        output='screen'
    )

    automatic_pick_place_node = Node(
        package='assembly_line',
        executable='automatic_pick_place',
        output='screen'
    )
    
    # -------------------------------------------------
    # TROLLEY / PALLET
    # -------------------------------------------------

    trolley_sdf = os.path.join(
        package_dir,
        'models',
        'trolley',
        'model.sdf'
    )

    spawn_trolley = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity',
            'trolley',
            '-file',
            trolley_sdf,
            '-x',
            '-0.8',
            '-y',
            '-1.4',
            '-z',
            '0.06'
        ],
        output='screen'
    )

    # -------------------------------------------------
    # CONTROLLERS
    # -------------------------------------------------

    
    conveyor_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'conveyor_velocity_controller',
        '--controller-manager',
        '/controller_manager',
        '--param-file',
            os.path.join(
                package_dir,
                'config',
                'conveyor_velocity_controller.yaml'
            ),
            '--namespace',
            '/'
        ],
        output='screen'
    )


    # -------------------------------------------------
    # DRILLING ROBOT CONTROLLERS (WORKSTATION 2)
    # -------------------------------------------------

    drilling_robot_joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/workstation2/controller_manager'
        ],
        output='screen'
    )

    drilling_robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'industrial_robot_controller',
            '--controller-manager',
            '/workstation2/controller_manager'
        ],
        output='screen'
    )


    # -------------------------------------------------
    # INDUSTRIAL ROBOT CONTROLLERS
    # -------------------------------------------------

    robot_joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/workstation1/controller_manager'
        ],
        output='screen'
    )

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'industrial_robot_controller',
            '--controller-manager',
            '/workstation1/controller_manager',
            '--param-file',
            robot_controllers_file
        ],
        output='screen'
    )
    
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'gripper_controller',
            '--controller-manager',
            '/workstation1/controller_manager',
            '--param-file',
            robot_controllers_file
        ],
        output='screen'
    )
    
    # -------------------------------------------------
    # ROBOT HOME POSITION
    # -------------------------------------------------

    robot_home_command = ExecuteProcess(
        cmd=[
            'ros2',
            'topic',
            'pub',
            '--once',
            '/workstation1/industrial_robot_controller/commands',
            'std_msgs/msg/Float64MultiArray',
            '{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'
        ],
        output='screen'
    )

    # -------------------------------------------------
    # WORKSTATION 2 TF CONNECTION
    # -------------------------------------------------

    workstation2_world_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'world',
            '--child-frame-id', 'workstation2_world'
        ],
        output='screen'
    )
    
    # -------------------------------------------------
    # LAUNCH
    # -------------------------------------------------

    return LaunchDescription([

        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=models_dir
        ),
        
        SetEnvironmentVariable(
            name='GAZEBO_PLUGIN_PATH',
            value=plugin_dir + ':' + os.environ.get('GAZEBO_PLUGIN_PATH', '')
        ),

        gazebo,
        workstation2_world_tf,

        # Conveyor
        conveyor_state_publisher,

        TimerAction(
            period=2.0,
            actions=[
                spawn_conveyor
            ]
        ),

        # Robot
        robot_state_publisher,
        
        # Drilling Robot (Workstation 2)
        drilling_robot_state_publisher,

        TimerAction(
            period=4.0,
            actions=[
                spawn_robot
            ]
        ),
        
        # Spawn Drilling Robot
        TimerAction(
            period=5.5,
            actions=[
                spawn_drilling_robot
            ]
        ),


        # Workpiece
        TimerAction(
            period=6.0,
            actions=[
                spawn_workpiece
            ]
        ),

        # Start workpiece automation
        TimerAction(
            period=12.0,
            actions=[
                workpiece_controller_node,
                workpiece_sensor_node
            ]
        ),
        
        # Trolley
        TimerAction(
            period=6.5,
            actions=[
                spawn_trolley
            ]
        ),

        
        TimerAction(
            period=9.0,
            actions=[
                conveyor_controller_spawner
            ]
        ),

        # Industrial robot controllers
        TimerAction(
            period=4.5,
            actions=[
                robot_joint_state_broadcaster_spawner
            ]
        ),

        TimerAction(
            period=8.0,
            actions=[
                robot_controller_spawner,
                gripper_controller_spawner,
            ]
        ),
        
        TimerAction(
            period=9.0,
            actions=[
                robot_home_command
            ]
        ),
        
        # Drilling Robot Controllers
        TimerAction(
            period=7.0,
            actions=[
                drilling_robot_joint_state_broadcaster_spawner
            ]
        ),

        TimerAction(
            period=10.0,
            actions=[
                drilling_robot_controller_spawner
            ]
        ),
        
        # Start robot automation
        TimerAction(
            period=11.0,
            actions=[
                drilling_controller_node,
                automatic_pick_place_node
            ]
        )

    ])
