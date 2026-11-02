import rclpy
import numpy as np
import subprocess
import time
import threading

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Bool, String

from .drill_ik_check import (
    solve_target,
    WORKPIECE_X,
    WORKPIECE_Y,
    TOP_Z,
    APPROACH_HEIGHT,
    DRILL_DEPTH,
    HOLES,
)


JOINT_NAMES = [
    'joint1', 'joint2', 'joint3',
    'joint4', 'joint5', 'joint6'
]

HOME = np.zeros(6)

CARTESIAN_STEP = 0.005
CARTESIAN_POINT_TIME = 0.15
JOINT_PATH_POINTS = 101
JOINT_PATH_DURATION = 5.0


class Robot2DrillingController(Node):

    def __init__(self):

        super().__init__('robot2_drilling_controller')

        # Separate node for trajectory execution
        # Separate node for trajectory execution
        self.motion_node = rclpy.create_node(
            'robot2_motion_client'
        )

        self.client = ActionClient(
            self.motion_node,
            FollowJointTrajectory,
            '/workstation2/industrial_robot_controller/follow_joint_trajectory'
        )

        # Drilling cycle state
        self.sequence_running = False
        self.sensor_clear = True

        # Publish completion after successful drilling and return HOME
        self.drilling_complete_publisher = self.create_publisher(
            Bool,
            '/drilling_complete',
            10
        )

        # Publish accuracy measurement events
        self.accuracy_event_publisher = self.create_publisher(
            String,
            '/accuracy_event',
            10
        )
        
        # Listen for workpiece detection
        self.sensor_subscription = self.create_subscription(
            Bool,
            '/workpiece_detected',
            self.sensor_callback,
            10
        )

        self.get_logger().info(
            'ROBOT 2 DRILLING CONTROLLER READY'
        )

        self.get_logger().info(
            'Waiting for workpiece detection...'
        )

    # -------------------------------------------------
    # WORKPIECE DETECTION
    # -------------------------------------------------

    def sensor_callback(self, msg):

        # Allow a new cycle after the sensor becomes clear
        if not msg.data:
            self.sensor_clear = True
            return

        # Ignore repeated detection messages
        if self.sequence_running or not self.sensor_clear:
            return

        self.sensor_clear = False
        self.sequence_running = True

        self.get_logger().info(
            'WORKPIECE DETECTED -> STARTING ROBOT 2 DRILLING'
        )

        threading.Thread(
            target=self.execute_drilling_cycle,
            daemon=True
        ).start()

    # -------------------------------------------------
    # EXECUTE DRILLING CYCLE
    # -------------------------------------------------

    def execute_drilling_cycle(self):

        success = self.run_sequence()

        if success:

            message = Bool()
            message.data = True

            self.drilling_complete_publisher.publish(message)

            self.get_logger().info(
                'DRILLING COMPLETE -> SIGNAL SENT TO ROBOT 1'
            )

        else:

            self.get_logger().error(
                'DRILLING FAILED. Robot 1 will not be triggered.'
            )

        self.sequence_running = False
    
    def interpolate_joint_path(self, start, end):

        return [
            start + t * (end - start)
            for t in np.linspace(0.0, 1.0, JOINT_PATH_POINTS)
        ]

    def calculate_vertical_path(self, hole, approach_q):

        dx, dy = hole

        x = WORKPIECE_X + dx
        y = WORKPIECE_Y + dy

        approach_z = TOP_Z + APPROACH_HEIGHT
        drilling_z = TOP_Z - DRILL_DEPTH

        count = int(
            round((approach_z - drilling_z) / CARTESIAN_STEP)
        )

        heights = np.linspace(
            approach_z,
            drilling_z,
            count + 1
        )

        configurations = [approach_q.copy()]
        previous_q = approach_q.copy()

        # Solve each Cartesian point before moving the robot.

        for z in heights[1:]:

            target = np.array([x, y, z])

            solution = solve_target(target, previous_q)

            if solution is None:

                self.get_logger().error(
                    f'IK failed at {target.tolist()}'
                )

                return None

            q = solution[0]

            configurations.append(q.copy())
            previous_q = q.copy()

        return configurations

    def make_trajectory_points(
        self,
        configurations,
        point_time
    ):

        points = []

        for index, q in enumerate(configurations, start=1):

            point = JointTrajectoryPoint()

            point.positions = q.tolist()

            total_time = index * point_time

            seconds = int(total_time)

            nanoseconds = int(
                round((total_time - seconds) * 1e9)
            )

            point.time_from_start = Duration(
                sec=seconds,
                nanosec=nanoseconds
            )

            points.append(point)

        return points

    def move_path(
        self,
        configurations,
        label,
        point_time
    ):

        self.get_logger().info(
            f'Starting: {label}'
        )

        goal = FollowJointTrajectory.Goal()

        goal.trajectory.joint_names = JOINT_NAMES

        goal.trajectory.points = self.make_trajectory_points(
            configurations,
            point_time
        )

        # Send trajectory goal
        send_future = self.client.send_goal_async(goal)

        goal_event = threading.Event()
        goal_result = {}

        def goal_response_callback(future):
            try:
                goal_result['handle'] = future.result()
            except Exception as error:
                goal_result['error'] = error
            finally:
                goal_event.set()

        send_future.add_done_callback(goal_response_callback)

        if not goal_event.wait(timeout=30.0):
            self.get_logger().error(
                f'Timeout waiting for goal response: {label}'
            )
            return False

        goal_handle = goal_result.get('handle')

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(
                f'Trajectory rejected: {label}'
            )
            return False

        # Wait for trajectory result
        result_future = goal_handle.get_result_async()

        result_event = threading.Event()
        result_data = {}

        def result_callback(future):
            try:
                result_data['result'] = future.result()
            except Exception as error:
                result_data['error'] = error
            finally:
                result_event.set()

        result_future.add_done_callback(result_callback)

        if not result_event.wait(timeout=120.0):
            self.get_logger().error(
                f'Timeout waiting for trajectory result: {label}'
            )
            return False

        wrapped_result = result_data.get('result')

        if wrapped_result is None:
            self.get_logger().error(
                f'No result received: {label}'
            )
            return False

        if wrapped_result.result.error_code != 0:
            self.get_logger().error(
                f'Trajectory failed: {label}; '
                f'error code: {wrapped_result.result.error_code}'
            )
            return False

        self.get_logger().info(
            f'Completed: {label}'
        )

        return True
    
    

    def show_drilled_workpiece(self):

        topic = '/gazebo/factory_world/visual'

        solid_message = (
            'name: "workpiece::workpiece_link::solid_visual" '
            'id: 108 '
            'parent_name: "workpiece::workpiece_link" '
            'parent_id: 107 '
            'transparency: 1.0'
        )

        drilled_message = (
            'name: "workpiece::workpiece_link::drilled_visual" '
            'id: 109 '
            'parent_name: "workpiece::workpiece_link" '
            'parent_id: 107 '
            'transparency: 0.0'
        )

        hide_solid = subprocess.run(
            ['gz', 'topic', '-p', topic, '-m', solid_message],
            capture_output=True,
            text=True,
            timeout=5
        )

        if hide_solid.returncode != 0:
            self.get_logger().error(
                f'Could not hide solid visual: {hide_solid.stderr.strip()}'
            )
            return False

        time.sleep(0.01)

        show_drilled = subprocess.run(
            ['gz', 'topic', '-p', topic, '-m', drilled_message],
            capture_output=True,
            text=True,
            timeout=5
        )

        if show_drilled.returncode != 0:
            self.get_logger().error(
                f'Could not show drilled visual: {show_drilled.stderr.strip()}'
            )

            # Restore the original solid appearance if mesh update fails.
            subprocess.run(
                [
                    'gz', 'topic', '-p', topic, '-m',
                    'name: "workpiece::workpiece_link::solid_visual" '
                    'id: 108 '
                    'parent_name: "workpiece::workpiece_link" '
                    'parent_id: 107 '
                    'transparency: 0.0'
                ],
                capture_output=True,
                text=True,
                timeout=5
            )

            return False

        self.get_logger().info(
            'Two drilled holes are now visible on the workpiece.'
        )

        return True
    
    def run_sequence(self):

        self.get_logger().info(
            'Waiting for Robot 2 trajectory controller...'
        )

        if not self.client.wait_for_server(timeout_sec=15.0):

            self.get_logger().error(
                'Robot 2 controller unavailable.'
            )

            return False

        # ------------------------------------------------
        # PRE-CALCULATE ALL PATHS BEFORE ROBOT MOVEMENT
        # ------------------------------------------------

        self.get_logger().info(
            'Calculating all Robot 2 trajectories...'
        )

        h1_x, h1_y = HOLES[0]

        h1_target = np.array([
            WORKPIECE_X + h1_x,
            WORKPIECE_Y + h1_y,
            TOP_Z + APPROACH_HEIGHT
        ])

        h1_solution = solve_target(h1_target, HOME)

        if h1_solution is None:
            self.get_logger().error('Hole 1 approach IK failed.')
            return False

        h1_approach = h1_solution[0]

        h1_down = self.calculate_vertical_path(
            HOLES[0],
            h1_approach
        )

        if h1_down is None:
            return False

        h1_up = list(reversed(h1_down))

        h2_x, h2_y = HOLES[1]

        h2_target = np.array([
            WORKPIECE_X + h2_x,
            WORKPIECE_Y + h2_y,
            TOP_Z + APPROACH_HEIGHT
        ])

        h2_solution = solve_target(
            h2_target,
            h1_approach
        )

        if h2_solution is None:
            self.get_logger().error('Hole 2 approach IK failed.')
            return False

        h2_approach = h2_solution[0]

        h2_down = self.calculate_vertical_path(
            HOLES[1],
            h2_approach
        )

        if h2_down is None:
            return False

        h2_up = list(reversed(h2_down))

        # These are the joint-space paths checked previously.
        home_to_h1 = self.interpolate_joint_path(
            HOME,
            h1_approach
        )

        h1_to_h2 = self.interpolate_joint_path(
            h1_approach,
            h2_approach
        )

        h2_to_home = self.interpolate_joint_path(
            h2_approach,
            HOME
        )

        self.get_logger().info(
            'All IK paths calculated successfully.'
        )

        # ------------------------------------------------
        # EXECUTE ROBOT 2 MOVEMENTS
        # ------------------------------------------------

        sequence = [

            (
                home_to_h1,
                'HOME TO HOLE 1 APPROACH',
                JOINT_PATH_DURATION / (len(home_to_h1) - 1)
            ),

            (
                h1_down,
                'HOLE 1 VERTICAL DRILLING MOVEMENT',
                CARTESIAN_POINT_TIME
            ),

            (
                h1_up,
                'HOLE 1 RETRACTION',
                CARTESIAN_POINT_TIME
            ),

            (
                h1_to_h2,
                'TRANSFER TO HOLE 2 APPROACH',
                JOINT_PATH_DURATION / (len(h1_to_h2) - 1)
            ),

            (
                h2_down,
                'HOLE 2 VERTICAL DRILLING MOVEMENT',
                CARTESIAN_POINT_TIME
            ),

            (
                h2_up,
                'HOLE 2 RETRACTION',
                CARTESIAN_POINT_TIME
            ),

            (
                h2_to_home,
                'RETURN TO HOME',
                JOINT_PATH_DURATION / (len(h2_to_home) - 1)
            ),
        ]

        for configurations, label, point_time in sequence:

            success = self.move_path(
                configurations,
                label,
                point_time
            )

            if not success:

                self.get_logger().error(
                    'Sequence stopped. Check Robot 2 in Gazebo.'
                )

                return False

            # Record Robot 2 drilling-depth measurement events
            if label == 'HOLE 1 VERTICAL DRILLING MOVEMENT':
                event = String()
                event.data = 'ROBOT2_HOLE1_DEPTH_REACHED'
                self.accuracy_event_publisher.publish(event)
                self.get_logger().info(
                    'Accuracy event published: ROBOT2_HOLE1_DEPTH_REACHED'
                )

            elif label == 'HOLE 2 VERTICAL DRILLING MOVEMENT':
                event = String()
                event.data = 'ROBOT2_HOLE2_DEPTH_REACHED'
                self.accuracy_event_publisher.publish(event)
                self.get_logger().info(
                    'Accuracy event published: ROBOT2_HOLE2_DEPTH_REACHED'
                )
            
            # Show the drilled holes after Robot 2 retracts from hole 2.
            if label == 'HOLE 2 RETRACTION':
            
                if not self.show_drilled_workpiece():
                    self.get_logger().error(
                    'Visual update failed. Stopping sequence before HOME.'
                    )
                    return False

        
        self.get_logger().info(
            'ROBOT 2 MOVEMENT SEQUENCE COMPLETED.'
        )

        self.get_logger().info(
            'Both drilling paths completed and Robot 2 returned HOME.'
        )

        self.get_logger().warn(
            'This was a movement test. No material was removed.'
        )
        return True


def main(args=None):

    rclpy.init(args=args)

    node = Robot2DrillingController()
    
    executor = MultiThreadedExecutor()
    
    executor.add_node(node)
    executor.add_node(node.motion_node)

    try:
        executor.spin()

    except KeyboardInterrupt:
        node.get_logger().warn('Drilling controller interrupted by user.')

    finally:
        executor.shutdown()
        node.motion_node.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
