#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from gazebo_msgs.srv import GetLinkState, SetModelState
from gazebo_msgs.msg import ModelState
from std_srvs.srv import SetBool


class WorkpieceAttacher(Node):

    def __init__(self):
        super().__init__('workpiece_attacher')

        self.attached = False
        self.busy = False

        self.get_link = self.create_client(
            GetLinkState,
            '/gazebo/get_link_state'
        )

        self.set_model = self.create_client(
            SetModelState,
            '/gazebo/set_model_state'
        )

        self.attach_service = self.create_service(
            SetBool,
            '/attach_workpiece',
            self.attach_callback
        )

        self.timer = self.create_timer(0.02, self.update)

        self.get_logger().info('WORKPIECE ATTACHER READY')
        self.get_logger().info(
            'Use: ros2 service call /attach_workpiece std_srvs/srv/SetBool "{data: true}"'
        )
        self.get_logger().info(
            'Detach: ros2 service call /attach_workpiece std_srvs/srv/SetBool "{data: false}"'
        )

    def attach_callback(self, request, response):

        if request.data:
            self.attached = True
            response.success = True
            response.message = 'Workpiece ATTACHED to gripper'
            self.get_logger().info('>>> WORKPIECE ATTACHED <<<')

        else:
            self.attached = False
            response.success = True
            response.message = 'Workpiece RELEASED'
            self.get_logger().info('>>> WORKPIECE RELEASED <<<')

        return response

    def update(self):

        if not self.attached:
            return

        if self.busy:
            return

        if not self.get_link.service_is_ready():
            return

        if not self.set_model.service_is_ready():
            return

        self.busy = True

        request = GetLinkState.Request()
        request.link_name = 'workstation1_robot::gripper_palm'
        request.reference_frame = 'world'

        future = self.get_link.call_async(request)
        future.add_done_callback(self.link_result)

    def link_result(self, future):

        try:
            result = future.result()

            if not result.success:
                self.get_logger().error(
                    'Cannot find gripper_palm: ' +
                    result.status_message
                )
                self.busy = False
                return

            palm = result.link_state

            state = ModelState()

            state.model_name = 'workpiece'

            # Keep the workpiece centered between the fingers
            state.pose.position.x = palm.pose.position.x
            state.pose.position.y = palm.pose.position.y
            state.pose.position.z = palm.pose.position.z - 0.12

            state.pose.orientation = palm.pose.orientation

            # Keep the object from receiving a large independent velocity
            state.twist = palm.twist

            state.reference_frame = 'world'

            request = SetModelState.Request()
            request.model_state = state

            future2 = self.set_model.call_async(request)
            future2.add_done_callback(self.set_result)

        except Exception as e:
            self.get_logger().error(str(e))
            self.busy = False

    def set_result(self, future):

        try:
            result = future.result()

            if not result.success:
                self.get_logger().error(
                    'Cannot move workpiece: ' +
                    result.status_message
                )

        except Exception as e:
            self.get_logger().error(str(e))

        self.busy = False


def main(args=None):

    rclpy.init(args=args)

    node = WorkpieceAttacher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
