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


class PositionAccuracyMonitorV2(Node):

    def __init__(self):

        super().__init__('position_accuracy_monitor_v2')

        # New CSV file. Existing accuracy files remain untouched.
        self.output_file = os.path.expanduser(
            '~/assembly_line_ws/position_accuracy_results_v2.csv'
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
                'workpiece_id',
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

        # The currently processed workpiece
        self.active_workpiece_id = None

        # Placement stability tracking
        self.placement_tracking_id = None
        self.settle_samples = []

        # Model states subscription
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

        self.settle_timer = self.create_timer(
            0.1,
            self.check_placement_stability
        )

        self.get_logger().info(
            'POSITION ACCURACY MONITOR V2 READY'
        )

        self.get_logger().info(
            f'CSV output: {self.output_file}'
        )

    # -------------------------------------------------
    # CSV WRITER
    # -------------------------------------------------

    def write_row(
        self,
        workpiece_id,
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
                workpiece_id or '',
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
    # ACCURACY EVENT CALLBACK
    # -------------------------------------------------

    def event_callback(self, msg):

        parts = msg.data.split('|', 1)
        event = parts[0]

        if event == 'SENSOR_WORKPIECE_DETECTED':

            if len(parts) != 2 or not parts[1]:

                self.get_logger().error(
                    'Sensor event did not contain a workpiece ID.'
                )
                return

            self.active_workpiece_id = parts[1]
            self.placement_tracking_id = None
            self.settle_samples = []

            self.get_logger().info(
                f'ACTIVE WORKPIECE: {self.active_workpiece_id}'
            )

            self.record_workpiece(event)
            return

        workpiece_id = self.active_workpiece_id

        if workpiece_id is None:

            self.get_logger().warning(
                f'Ignoring {event}: no active workpiece ID.'
            )
            return

        self.get_logger().info(
            f'Accuracy event: {event} | Workpiece: {workpiece_id}'
        )

        if event == 'ROBOT1_PICK_POSITION_REACHED':

            self.record_workpiece(event)

            self.request_entity_pose(
                event,
                'workstation1_robot::link6',
                'robot1_link6',
                workpiece_id
            )

        elif event == 'ROBOT1_RELEASE_POSITION_REACHED':

            self.record_workpiece(event)

            self.request_entity_pose(
                event,
                'workstation1_robot::link6',
                'robot1_link6',
                workpiece_id
            )

            self.placement_tracking_id = workpiece_id
            self.settle_samples = []

            self.get_logger().info(
                f'Started placement tracking for {workpiece_id}'
            )

        elif event == 'ROBOT2_HOLE1_DEPTH_REACHED':

            self.request_entity_pose(
                event,
                'workstation2_robot::link6',
                'robot2_hole1',
                workpiece_id
            )

        elif event == 'ROBOT2_HOLE2_DEPTH_REACHED':

            self.request_entity_pose(
                event,
                'workstation2_robot::link6',
                'robot2_hole2',
                workpiece_id
            )
        # All other events belong to the currently active workpiece.

        workpiece_id = self.active_workpiece_id

        if workpiece_id is None:

            self.get_logger().warning(
                f'Ignoring {event}: no active workpiece ID.'
            )

            return

        self.get_logger().info(
            f'Accuracy event: {event} | '
            f'Workpiece: {workpiece_id}'
        )

        if event in [
            'ROBOT1_PICK_POSITION_REACHED',
            'ROBOT1_RELEASE_POSITION_REACHED'
        ]:

            self.record_workpiece(event)

            self.request_entity_pose(
                event,
                'workstation1_robot::link6',
                'robot1_link6',
                workpiece_id
            )

        elif event == 'ROBOT1_RELEASE_POSITION_REACHED':

            self.placement_tracking_id = workpiece_id
            self.settle_samples = []

            self.get_logger().info(
                f'Started placement tracking for {workpiece_id}'
            )

        elif event == 'ROBOT2_HOLE1_DEPTH_REACHED':

            self.request_entity_pose(
                event,
                'workstation2_robot::link6',
                'robot2_hole1',
                workpiece_id
            )

        elif event == 'ROBOT2_HOLE2_DEPTH_REACHED':

            self.request_entity_pose(
                event,
                'workstation2_robot::link6',
                'robot2_hole2',
                workpiece_id
            )

    # -------------------------------------------------
    # RECORD ACTIVE WORKPIECE POSITION
    # -------------------------------------------------

    def record_workpiece(self, event):

        workpiece_id = self.active_workpiece_id

        if workpiece_id is None:
            return

        state = self.workpiece_states.get(workpiece_id)

        if state is None:

            self.write_row(
                workpiece_id,
                event,
                workpiece_id,
                'workpiece_position',
                notes='Workpiece pose unavailable in latest ModelStates'
            )

            self.get_logger().warning(
                f'Pose unavailable for {workpiece_id}'
            )

            return

        self.write_row(
            workpiece_id,
            event,
            workpiece_id,
            'workpiece_position',
            state['position'],
            state['orientation']
        )

        self.get_logger().info(
            f'{workpiece_id} position: '
            f'{state["position"]}'
        )

    # -------------------------------------------------
    # QUERY ACTUAL GAZEBO LINK POSE
    # -------------------------------------------------

    def request_entity_pose(
        self,
        event,
        entity,
        measurement,
        workpiece_id
    ):

        if not self.entity_client.service_is_ready():

            self.get_logger().error(
                'Gazebo GetEntityState service unavailable.'
            )

            self.write_row(
                workpiece_id,
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
                measurement,
                workpiece_id
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
        measurement,
        workpiece_id
    ):

        try:

            response = future.result()

            if not response.success:

                self.get_logger().error(
                    f'Could not retrieve pose: {entity}'
                )

                self.write_row(
                    workpiece_id,
                    event,
                    entity,
                    measurement,
                    notes='Gazebo entity-state request failed'
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
                workpiece_id,
                event,
                entity,
                measurement,
                position,
                orientation,
                gazebo_stamp
            )

            self.get_logger().info(
                f'{workpiece_id} | {measurement}: '
                f'X={position[0]:.6f}, '
                f'Y={position[1]:.6f}, '
                f'Z={position[2]:.6f}'
            )

            # Calculate estimated drill-tip position
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
                    workpiece_id,
                    event,
                    entity,
                    measurement + '_estimated_drill_tip',
                    tip_position,
                    notes=(
                        'Estimated from link6 pose and '
                        '0.33 m local-Z offset'
                    )
                )

                self.get_logger().info(
                    f'{workpiece_id} estimated drill tip: '
                    f'{tip_position}'
                )

        except Exception as error:

            self.get_logger().error(
                f'Pose measurement error: {error}'
            )

    # -------------------------------------------------
    # PLACEMENT STABILITY DETECTION
    # -------------------------------------------------

    def check_placement_stability(self):

        workpiece_id = self.placement_tracking_id

        if workpiece_id is None:
            return

        state = self.workpiece_states.get(workpiece_id)

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

        # Require variation below 1.5 mm
        if maximum_deviation <= 0.0015:

            self.write_row(
                workpiece_id,
                'PLACEMENT_SETTLED',
                workpiece_id,
                'settled_workpiece_position',
                mean_position,
                notes=(
                    'Stable over approximately 1.5 seconds; '
                    f'maximum deviation={maximum_deviation:.6f} m'
                )
            )

            self.get_logger().info(
                f'PLACEMENT STABLE: {workpiece_id} | '
                f'Position: {mean_position}'
            )

            self.placement_tracking_id = None
            self.settle_samples = []

    # -------------------------------------------------
    # CLEANUP
    # -------------------------------------------------

    def destroy_node(self):

        if hasattr(self, 'csv_file') and not self.csv_file.closed:
            self.csv_file.close()

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = PositionAccuracyMonitorV2()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
