import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class RobotPickPlace(Node):

    def __init__(self):
        super().__init__('robot_pick_place')

        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/workstation1/industrial_robot_controller/commands',
            10
        )

        # Joint positions in radians
        self.positions = [
            # 1. Home
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],

            # 2. Move above the pick area
            [0.5, -0.3, 0.2, 0.0, 0.0, 0.0],

            # 3. Move down toward the pick area
            [0.5, -0.5, 0.4, 0.0, 0.0, 0.0],

            # 4. Lift the object
            [0.5, -0.3, 0.2, 0.0, 0.0, 0.0],

            # 5. Move above the place area
            [-0.5, -0.3, 0.2, 0.0, 0.0, 0.0],

            # 6. Move down toward the place area
            [-0.5, -0.5, 0.4, 0.0, 0.0, 0.0],

            # 7. Return upward
            [-0.5, -0.3, 0.2, 0.0, 0.0, 0.0],

            # 8. Return home
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ]

        self.index = 0

        self.timer = self.create_timer(
            3.0,
            self.send_position
        )

    def send_position(self):
        msg = Float64MultiArray()
        msg.data = self.positions[self.index]

        self.publisher.publish(msg)

        self.get_logger().info(
            f'Step {self.index + 1}/{len(self.positions)}: '
            f'{list(msg.data)}'
        )

        self.index += 1

        if self.index >= len(self.positions):
            self.index = 0


def main(args=None):
    rclpy.init(args=args)

    node = RobotPickPlace()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
