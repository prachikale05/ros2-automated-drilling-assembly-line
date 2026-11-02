import os
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState, SpawnEntity
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool, Float64MultiArray


class WorkpieceController(Node):

    def __init__(self):
        super().__init__('workpiece_controller')

        # Active workpiece and conveyor settings
        self.active_workpiece = 'workpiece'
        self.next_workpiece_number = 2

        self.workpiece_x = -1.2
        self.workpiece_y = 0.0
        self.workpiece_z = 0.87

        self.speed = 0.25
        self.current_conveyor_velocity = 0.5
        self.running = True

        self.last_detection = False
        self.spawn_pending = False

        # Existing conveyor topic
        self.conveyor_publisher = self.create_publisher(
            Float64MultiArray,
            '/conveyor_velocity_controller/commands',
            10
        )

        # Existing Gazebo position service
        self.state_client = self.create_client(
            SetEntityState,
            '/set_entity_state'
        )

        # Gazebo service for spawning additional workpieces
        self.spawn_client = self.create_client(
            SpawnEntity,
            '/spawn_entity'
        )

        # Reuse the existing model, including its physics and plugin
        model_path = os.path.expanduser(
            '~/assembly_line_ws/src/assembly_line/models/workpiece/model.sdf'
        )

        with open(model_path, 'r') as model_file:
            self.model_xml = model_file.read()

        # Sensor detection
        self.sensor_subscription = self.create_subscription(
            Bool,
            '/workpiece_detected',
            self.sensor_callback,
            10
        )

        # The robot will publish True after completing its cycle
        self.cycle_subscription = self.create_subscription(
            Bool,
            '/workpiece_cycle_complete',
            self.cycle_complete_callback,
            10
        )

        # Timers
        self.conveyor_timer = self.create_timer(
            0.1,
            self.publish_conveyor_command
        )

        self.workpiece_timer = self.create_timer(
            0.05,
            self.move_workpiece
        )

        self.get_logger().info(
            'WORKPIECE CONTROLLER STARTED'
        )

        self.get_logger().info(
            'Waiting for workpiece detection and cycle completion'
        )

    # -------------------------------------------------
    # CONVEYOR
    # -------------------------------------------------

    def publish_conveyor_command(self):
        message = Float64MultiArray()
        message.data = [
            self.current_conveyor_velocity,
            self.current_conveyor_velocity
        ]
        self.conveyor_publisher.publish(message)

    def set_conveyor_velocity(self, velocity):
        self.current_conveyor_velocity = velocity
        self.publish_conveyor_command()

    # -------------------------------------------------
    # MOVE ONLY THE CURRENT ACTIVE WORKPIECE
    # -------------------------------------------------

    def move_workpiece(self):
        if not self.running:
            return

        self.workpiece_x += self.speed * 0.05

        if self.workpiece_x >= 1.2:
            self.workpiece_x = -1.2
            self.get_logger().info(
                'ACTIVE WORKPIECE RETURNED TO START'
            )

        if not self.state_client.service_is_ready():
            return

        request = SetEntityState.Request()
        state = EntityState()

        state.name = self.active_workpiece
        state.pose.position.x = self.workpiece_x
        state.pose.position.y = self.workpiece_y
        state.pose.position.z = self.workpiece_z
        state.pose.orientation.w = 1.0

        state.twist.linear.x = 0.0
        state.twist.linear.y = 0.0
        state.twist.linear.z = 0.0

        request.state = state
        self.state_client.call_async(request)

    # -------------------------------------------------
    # SENSOR
    # -------------------------------------------------

    def sensor_callback(self, msg):
        detected = msg.data

        if not detected:
            self.last_detection = False
            return

        if self.last_detection:
            return

        self.last_detection = True

        if not self.running:
            return

        self.get_logger().info(
            f'{self.active_workpiece} detected; stopping conveyor'
        )

        self.running = False
        self.set_conveyor_velocity(0.0)

    # -------------------------------------------------
    # ROBOT CYCLE COMPLETION
    # -------------------------------------------------

    def cycle_complete_callback(self, msg):
        if not msg.data:
            return

        if self.spawn_pending:
            self.get_logger().info(
                'A workpiece spawn is already in progress'
            )
            return

        self.get_logger().info(
            'Robot cycle complete; preparing next workpiece'
        )

        self.spawn_next_workpiece()

    # -------------------------------------------------
    # SPAWN NEXT WORKPIECE
    # -------------------------------------------------

    def spawn_next_workpiece(self):
        if not self.spawn_client.service_is_ready():
            self.get_logger().error(
                '/spawn_entity is not available; cannot spawn next workpiece'
            )
            return

        new_name = f'workpiece_{self.next_workpiece_number}'

        request = SpawnEntity.Request()
        request.name = new_name
        request.xml = self.model_xml
        request.robot_namespace = ''
        request.reference_frame = 'world'

        request.initial_pose = Pose()
        request.initial_pose.position.x = -1.2
        request.initial_pose.position.y = 0.0
        request.initial_pose.position.z = 0.87
        request.initial_pose.orientation.w = 1.0

        self.spawn_pending = True

        self.get_logger().info(
            f'Spawning {new_name} at the conveyor starting position'
        )

        future = self.spawn_client.call_async(request)

        future.add_done_callback(
            lambda result: self.spawn_response_callback(
                result, new_name
            )
        )

    def spawn_response_callback(self, future, new_name):
        self.spawn_pending = False

        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(
                f'Spawn service failed: {error}'
            )
            return

        if not response.success:
            self.get_logger().error(
                f'Failed to spawn {new_name}: {response.status_message}'
            )
            return

        # Change the active entity only after successful spawning.
        # Previously placed workpieces are never selected for movement.
        self.active_workpiece = new_name
        self.next_workpiece_number += 1

        self.workpiece_x = -1.2
        self.workpiece_y = 0.0
        self.workpiece_z = 0.87

        self.last_detection = False
        self.running = True

        self.set_conveyor_velocity(0.5)

        self.get_logger().info(
            f'{new_name} spawned successfully; conveyor restarted'
        )


def main(args=None):
    rclpy.init(args=args)

    node = WorkpieceController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
