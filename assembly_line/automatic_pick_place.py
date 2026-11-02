import rclpy
from rclpy.node import Node

from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Bool, Float64MultiArray, String


class AutomaticPickPlace(Node):

    def __init__(self):
        super().__init__('automatic_pick_place')

        # -------------------------------------------------
        # ROBOT TRAJECTORY ACTION
        # -------------------------------------------------

        self.robot_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/workstation1/industrial_robot_controller/follow_joint_trajectory'
        )

        self.gripper_publisher = self.create_publisher(
            Float64MultiArray,
            '/workstation1/gripper_controller/commands',
            10
        )

        # -------------------------------------------------
        # CONVEYOR
        # -------------------------------------------------

        self.conveyor_publisher = self.create_publisher(
            Float64MultiArray,
            '/workstation1/conveyor_velocity_controller/commands',
            10
        )

        # -------------------------------------------------
        # SENSOR
        # -------------------------------------------------

        self.drilling_subscription = self.create_subscription(
            Bool,
            '/drilling_complete',
            self.drilling_complete_callback,
            10
        )

        # Notify the workpiece controller when the robot finishes a cycle
        self.cycle_complete_publisher = self.create_publisher(
            Bool,
            '/workpiece_cycle_complete',
            10
        )

        # Publish position accuracy measurement events
        self.accuracy_event_publisher = self.create_publisher(
            String,
            '/accuracy_event',
            10
        )
        
        # -------------------------------------------------
        # VERIFIED ROBOT POSITIONS
        # -------------------------------------------------

        self.home = [
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        ]

        # Verified successful pickup position
        self.pick = [
            0.94, -1.60, 1.52, 0.00, 0.10, 0.75
        ]

        # Verified successful trolley alignment
        self.trolley = [
            -0.70, -1.30, 1.20, 0.00, 0.10, -0.70
        ]

        # Verified successful release position
        self.release = [
            -1.08, -1.30, 1.70, 0.00, 0.60, -0.70
        ]

        # -------------------------------------------------
        # STATE
        # -------------------------------------------------

        self.sequence_running = False
        self.detected = False
        self.sensor_clear = True

        self.get_logger().info(
            'AUTOMATIC PICK-AND-PLACE NODE STARTED'
        )

        self.get_logger().info(
            'Waiting for workpiece detection...'
        )

    # -------------------------------------------------
    # SENSOR CALLBACK
    # -------------------------------------------------

    def drilling_complete_callback(self, msg):

        if not msg.data:
            return

        # Ignore duplicate signals while pick-and-place is running
        if self.sequence_running:
            return
        
        self.sequence_running = True

        self.get_logger().info(
            'DRILLING COMPLETED -> STARTING ROBOT 1 PICK-AND-PLACE'
        )

        self.start_sequence()

    # -------------------------------------------------
    # START SEQUENCE
    # -------------------------------------------------

    def start_sequence(self):

        self.get_logger().info(
            'Waiting for robot trajectory controller...'
        )

        if not self.robot_client.wait_for_server(
            timeout_sec=5.0
        ):
            self.get_logger().error(
                'Robot trajectory action server not available'
            )

            self.sequence_running = False
            return

        self.get_logger().info(
            'Robot controller connected'
        )

        # Make sure conveyor remains stopped
        self.stop_conveyor()

        # Open gripper before approaching workpiece
        self.open_gripper()

        # Give gripper time to open
        timer = self.create_timer(
            0.4,
            self.move_to_pick,
            callback_group=None
        )
        timer.cancel()

        self._pick_timer = timer
        self._pick_timer.reset()

    # -------------------------------------------------
    # MOVE TO PICK
    # -------------------------------------------------

    def move_to_pick(self):

        if hasattr(self, '_pick_timer'):
            self._pick_timer.cancel()

        self.get_logger().info(
            'Moving robot to PICK position'
        )

        self.send_robot_position(
            self.pick,
            2.5,
            self.after_pick_position
        )

    # -------------------------------------------------
    # AFTER PICK POSITION
    # -------------------------------------------------

    def after_pick_position(self):

        self.get_logger().info(
            'Robot reached PICK position'
        )

        event = String()
        event.data = 'ROBOT1_PICK_POSITION_REACHED'
        self.accuracy_event_publisher.publish(event)

        self.get_logger().info(
            'Accuracy event published: ROBOT1_PICK_POSITION_REACHED'
        )
        
        self.get_logger().info(
            'Closing gripper'
        )

        self.close_gripper()

        # Wait before lifting
        timer = self.create_timer(
            0.4,
            self.lift_after_pick,
            callback_group=None
        )
        timer.cancel()

        self._lift_timer = timer
        self._lift_timer.reset()

    # -------------------------------------------------
    # LIFT
    # -------------------------------------------------

    def lift_after_pick(self):

        if hasattr(self, '_lift_timer'):
            self._lift_timer.cancel()

        self.get_logger().info(
            'Lifting workpiece'
        )

        self.send_robot_position(
            self.pick,
            1.0,
            self.move_to_trolley
        )

    # -------------------------------------------------
    # MOVE TO TROLLEY
    # -------------------------------------------------

    def move_to_trolley(self):

        self.get_logger().info(
            'Moving workpiece toward trolley'
        )

        self.send_robot_position(
            self.trolley,
            3.0,
            self.move_to_release
        )

    # -------------------------------------------------
    # MOVE TO RELEASE
    # -------------------------------------------------

    def move_to_release(self):

        self.get_logger().info(
            'Moving to RELEASE position'
        )

        self.send_robot_position(
            self.release,
            3.5,
            self.release_workpiece
        )

    # -------------------------------------------------
    # RELEASE
    # -------------------------------------------------

    def release_workpiece(self):

        self.get_logger().info(
            'Opening gripper -> releasing workpiece'
        )

        self.open_gripper()
        
        event = String()
        event.data = 'ROBOT1_RELEASE_POSITION_REACHED'
        self.accuracy_event_publisher.publish(event)

        self.get_logger().info(
            'Accuracy event published: ROBOT1_RELEASE_POSITION_REACHED'
        )
        
        timer = self.create_timer(
            1.0,
            self.finish_cycle,
            callback_group=None
        )
        timer.cancel()

        self._finish_timer = timer
        self._finish_timer.reset()

    # -------------------------------------------------
    # FINISH
    # -------------------------------------------------

    def finish_cycle(self):

        if hasattr(self, '_finish_timer'):
            self._finish_timer.cancel()

        self.get_logger().info(
            'WORKPIECE RELEASED - MOVING ROBOT AWAY'
        )

        # Safe position above the trolley
        safe_retreat = [
            -0.70, -1.30, 1.20, 0.00, 0.10, -0.70
        ]
        
        self.send_robot_position(
            safe_retreat,
            3.0,
            self.return_home
        )

    # -------------------------------------------------
    # RETURN HOME
    # -------------------------------------------------
    
    def return_home(self):

        self.get_logger().info(
            'WORKPIECE PLACED IN TROLLEY'
        )

        self.get_logger().info(
            'Returning robot HOME'
        )

        self.send_robot_position(
            self.home,
            1.8,
            self.restart_conveyor
        )
    
    # -------------------------------------------------
    # RESTART CONVEYOR
    # -------------------------------------------------

    def restart_conveyor(self):

        self.get_logger().info(
            'ROBOT HOME REACHED'
        )

        self.get_logger().info(
            'REQUESTING NEXT WORKPIECE'
        )

        message = Bool()
        message.data = True
        self.cycle_complete_publisher.publish(message)

        self.sequence_running = False
        self.detected = False

        self.get_logger().info(
            'AUTOMATIC CYCLE COMPLETE'
        )

        self.get_logger().info(
            'WAITING FOR NEXT WORKPIECE'
        )

    # -------------------------------------------------
    # SEND ROBOT TRAJECTORY
    # -------------------------------------------------

    def send_robot_position(
        self,
        positions,
        duration,
        callback
    ):

        goal_msg = FollowJointTrajectory.Goal()

        goal_msg.trajectory.joint_names = [
            'joint1',
            'joint2',
            'joint3',
            'joint4',
            'joint5',
            'joint6'
        ]

        point = JointTrajectoryPoint()

        point.positions = positions

        point.time_from_start.sec = int(duration)

        goal_msg.trajectory.points.append(point)

        send_goal_future = self.robot_client.send_goal_async(
            goal_msg
        )

        send_goal_future.add_done_callback(
            lambda future: self.goal_response_callback(
                future,
                callback
            )
        )

    # -------------------------------------------------
    # GOAL RESPONSE
    # -------------------------------------------------

    def goal_response_callback(
        self,
        future,
        callback
    ):

        goal_handle = future.result()

        if not goal_handle.accepted:

            self.get_logger().error(
                'Robot trajectory goal rejected'
            )

            self.sequence_running = False
            return

        self.get_logger().info(
            'Robot trajectory accepted'
        )

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            lambda future: callback()
        )

    # -------------------------------------------------
    # GRIPPER
    # -------------------------------------------------

    def open_gripper(self):

        message = Float64MultiArray()

        message.data = [
            0.12,
            -0.12
        ]

        self.gripper_publisher.publish(message)

    def close_gripper(self):

        message = Float64MultiArray()

        message.data = [
            0.0,
            0.0
        ]

        self.gripper_publisher.publish(message)

    # -------------------------------------------------
    # CONVEYOR
    # -------------------------------------------------

    def stop_conveyor(self):

        message = Float64MultiArray()

        message.data = [
            0.0,
            0.0
        ]

        self.conveyor_publisher.publish(message)

    def start_conveyor(self):

        message = Float64MultiArray()

        message.data = [
            0.5,
            0.5
        ]

        self.conveyor_publisher.publish(message)


def main(args=None):

    rclpy.init(args=args)

    node = AutomaticPickPlace()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
