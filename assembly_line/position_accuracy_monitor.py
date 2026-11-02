import csv
import os
import time
import threading
from datetime import datetime

import numpy as np
import rclpy

from rclpy.node import Node
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetEntityState


class PositionAccuracyMonitor(Node):

    def __init__(self):

        super().__init__('position_accuracy_monitor')

        self.output_file = os.path.expanduser(
            '~/assembly_line_ws/position_accuracy_results.csv'
        )

        self.file_lock = threading.Lock()

        self.csv_file = open(
            self.output_file,
            'a',
            newline=''
        )

        self.writer = csv.writer(self.csv_file)

        if os.path.getsize(self.output_file) == 0:

            self.writer.writerow([
                'wall_time',
                'event',
                'entity',
                'measurement',
                'frame',
                'x_m',
                'y_m',
                'z_m',
                'qx',
                'qy',
                'qz',
                'qw',
                'gazebo_stamp',
                'notes'
            ])

            self.csv_file.flush()

        # Latest workpiece poses from Gazebo
        self.workpiece_states = {}

        self.model_subscription = self.create_subscription(
            ModelStates,
            '/model_states',
            self.model_states_callback,
            10
        )

        # Accuracy event subscription
        self.event_subscription = self.create_subscription(
            String,
            '/accuracy_event',
            self.event_callback,
            10
        )

        # Gazebo entity-state service
        self.entity_client = self.create_client(
            GetEntityState,
            '/get_entity_state'
        )

        # Workpiece placement stability tracking
        self.track_placement = False
        self.settle_samples = []

        self.settle_timer = self.create_timer(
            0.1,
            self.check_placement_stability
        )

        self.get_logger().info(
            'POSITION ACCURACY MONITOR READY'
        )

        self.get_logger().info(
            f'CSV output: {self.output_file}'
        )

    # -------------------------------------------------
    # CSV WRITER
    # -------------------------------------------------

    def write_row(
        self,
        event,
        entity,
        measurement,
        position=None,
        orientation=None,
        gazebo_stamp='',
        notes=''
    ):

        if position is None:
            position = ['', '', '']

        if orientation is None:
            orientation = ['', '', '', '']

        with self.file_lock:

            self.writer.writerow([
                datetime.now().isoformat(timespec='milliseconds'),
                event,
                entity,
                measurement,
                'world',
                position[0],
                position[1],
                position[2],
                orientation[0],
                orientation[1],
                orientation[2],
                orientation[3],
                gazebo_stamp,
                notes
            ])

            self.csv_file.flush()

    # -------------------------------------------------
    # WORKPIECE MODEL STATES
    # -------------------------------------------------

    def model_states_callback(self, msg):

        states = {}

        for index, name in enumerate(msg.name):

            if name != 'workpiece' and not name.startswith(
                'workpiece_'
            ):
                continue

            pose = msg.pose[index]

            states[name] = {
                'position': np.array([
                    pose.position.x,
                    pose.position.y,
                    pose.position.z
                ]),
                'orientation': [
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w
                ]
            }

        self.workpiece_states = states

    # -------------------------------------------------
    # EVENT CALLBACK
    # -------------------------------------------------

    def event_callback(self, msg):

        event = msg.data

        self.get_logger().info(
            f'Accuracy event received: {event}'
        )

        if event in [
            'SENSOR_WORKPIECE_DETECTED',
            'ROBOT1_PICK_POSITION_REACHED',
            'ROBOT1_RELEASE_POSITION_REACHED'
        ]:

            self.record_workpiece(event)

        if event == 'ROBOT1_PICK_POSITION_REACHED':

            self.request_entity_pose(
                event,
                'workstation1_robot::link6',
                'robot1_link6'
            )

        elif event == 'ROBOT1_RELEASE_POSITION_REACHED':

            self.request_entity_pose(
                event,
                'workstation1_robot::link6',
                'robot1_link6'
            )

            self.track_placement = True
            self.settle_samples = []

        elif event == 'ROBOT2_HOLE1_DEPTH_REACHED':

            self.request_entity_pose(
                event,
                'workstation2_robot::link6',
                'robot2_hole1'
            )

        elif event == 'ROBOT2_HOLE2_DEPTH_REACHED':

            self.request_entity_pose(
                event,
                'workstation2_robot::link6',
                'robot2_hole2'
            )

    # -------------------------------------------------
    # RECORD WORKPIECE POSITION
    # -------------------------------------------------

    def record_workpiece(self, event):

        if not self.workpiece_states:

            self.write_row(
                event,
                'workpiece',
                'workpiece_position',
                notes='Workpiece pose unavailable at event time'
            )

            return

        for name, state in self.workpiece_states.items():

            self.write_row(
                event,
                name,
                'workpiece_position',
                state['position'],
                state['orientation']
            )

    # -------------------------------------------------
    # QUERY ACTUAL GAZEBO LINK POSE
    # -------------------------------------------------

    def request_entity_pose(
        self,
        event,
        entity,
        measurement
    ):

        if not self.entity_client.service_is_ready():

            self.get_logger().error(
                'Gazebo GetEntityState service unavailable.'
            )

            self.write_row(
                event,
                entity,
                measurement,
                notes='Gazebo service unavailable'
            )

            return

        request = GetEntityState.Request()

        request.name = entity
        request.reference_frame = 'world'

        future = self.entity_client.call_async(request)

        future.add_done_callback(
            lambda result: self.entity_response_callback(
                result,
                event,
                entity,
                measurement
            )
        )

    # -------------------------------------------------
    # PROCESS GAZEBO RESPONSE
    # -------------------------------------------------

    def entity_response_callback(
        self,
        future,
        event,
        entity,
        measurement
    ):

        try:

            response = future.result()

            if not response.success:

                self.get_logger().error(
                    f'Could not retrieve pose: {entity}'
                )

                return

            pose = response.state.pose

            position = np.array([
                pose.position.x,
                pose.position.y,
                pose.position.z
            ])

            orientation = np.array([
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w
            ])

            gazebo_stamp = (
                f'{response.header.stamp.sec}.'
                f'{response.header.stamp.nanosec:09d}'
            )

            self.write_row(
                event,
                entity,
                measurement,
                position,
                orientation,
                gazebo_stamp
            )

            self.get_logger().info(
                f'{measurement}: '
                f'X={position[0]:.5f}, '
                f'Y={position[1]:.5f}, '
                f'Z={position[2]:.5f}'
            )

            # Calculate drill-tip position from actual link6 pose
            if measurement.startswith('robot2_hole'):

                x, y, z, w = orientation

                local_z_direction = np.array([
                    2.0 * (x * z + w * y),
                    2.0 * (y * z - w * x),
                    1.0 - 2.0 * (x * x + y * y)
                ])

                tip_position = (
                    position + 0.33 * local_z_direction
                )

                self.write_row(
                    event,
                    entity,
                    measurement + '_estimated_drill_tip',
                    tip_position,
                    notes='Calculated from measured link6 pose and 0.33 m local-Z offset'
                )

                self.get_logger().info(
                    f'Estimated drill tip: '
                    f'{tip_position}'
                )

        except Exception as error:

            self.get_logger().error(
                f'Pose measurement error: {error}'
            )

    # -------------------------------------------------
    # WORKPIECE SETTLEMENT DETECTION
    # -------------------------------------------------

    def check_placement_stability(self):

        if not self.track_placement:
            return

        if not self.workpiece_states:
            return

        state = self.workpiece_states.get('workpiece')

        if state is None:
            return

        now = time.monotonic()

        self.settle_samples.append((
            now,
            state['position'].copy()
        ))

        # Keep the latest 1.5 seconds
        self.settle_samples = [
            sample for sample in self.settle_samples
            if now - sample[0] <= 1.5
        ]

        if not self.settle_samples:
            return

        elapsed = (
            self.settle_samples[-1][0]
            - self.settle_samples[0][0]
        )

        if elapsed < 1.4:
            return

        positions = np.array([
            sample[1] for sample in self.settle_samples
        ])

        mean_position = np.mean(positions, axis=0)

        deviations = np.linalg.norm(
            positions - mean_position,
            axis=1
        )

        maximum_deviation = np.max(deviations)

        # Require position variation below 1.5 mm
        if maximum_deviation <= 0.0015:

            self.write_row(
                'PLACEMENT_SETTLED',
                'workpiece',
                'settled_workpiece_position',
                mean_position,
                notes=(
                    'Stable over 1.5 seconds; '
                    f'maximum deviation={maximum_deviation:.6f} m'
                )
            )

            self.get_logger().info(
                'WORKPIECE PLACEMENT STABLE - POSITION RECORDED'
            )

            self.track_placement = False
            self.settle_samples = []

    # -------------------------------------------------
    # CLEANUP
    # -------------------------------------------------

    def destroy_node(self):

        if hasattr(self, 'csv_file'):
            self.csv_file.close()

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = PositionAccuracyMonitor()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
