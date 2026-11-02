import rclpy
from rclpy.node import Node

from gazebo_msgs.srv import GetEntityState
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose


class ConveyorWorkpiece(Node):

    def __init__(self):
        super().__init__('conveyor_workpiece')

        self.get_state = self.create_client(
            GetEntityState,
            '/gazebo/get_entity_state'
        )

        self.set_state = self.create_client(
            SetEntityState,
            '/gazebo/set_entity_state'
        )

        self.x = -1.2
        self.speed = 0.5
        self.last_time = self.get_clock().now()

        self.timer = self.create_timer(0.05, self.update)

        self.get_logger().info('Conveyor workpiece controller started.')

    def update(self):

        if not self.get_state.service_is_ready():
            return

        if not self.set_state.service_is_ready():
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        self.x += self.speed * dt

        # End of conveyor
        if self.x > 1.75:
            self.x = -1.75

        state = EntityState()
        state.name = 'workpiece'
        state.reference_frame = 'world'

        state.pose.position.x = self.x
        state.pose.position.y = 0.0
        state.pose.position.z = 1.05

        state.pose.orientation.x = 0.0
        state.pose.orientation.y = 0.0
        state.pose.orientation.z = 0.0
        state.pose.orientation.w = 1.0

        request = SetEntityState.Request()
        request.state = state

        self.set_state.call_async(request)


def main(args=None):

    rclpy.init(args=args)

    node = ConveyorWorkpiece()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
