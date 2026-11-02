import numpy as np
from math import sin, cos, pi


def translation(x, y, z):
    T = np.eye(4)
    T[:3, 3] = [x, y, z]
    return T


def rotation_x(a):
    T = np.eye(4)
    T[:3, :3] = [
        [1, 0, 0],
        [0, cos(a), -sin(a)],
        [0, sin(a), cos(a)]
    ]
    return T


def rotation_y(a):
    T = np.eye(4)
    T[:3, :3] = [
        [cos(a), 0, sin(a)],
        [0, 1, 0],
        [-sin(a), 0, cos(a)]
    ]
    return T


def rotation_z(a):
    T = np.eye(4)
    T[:3, :3] = [
        [cos(a), -sin(a), 0],
        [sin(a), cos(a), 0],
        [0, 0, 1]
    ]
    return T


def rpy(roll, pitch, yaw):
    return rotation_z(yaw) @ rotation_y(pitch) @ rotation_x(roll)


def joint_rotation(axis, angle):
    if axis == 'x':
        return rotation_x(angle)
    if axis == 'y':
        return rotation_y(angle)
    return rotation_z(angle)


def calculate_fk(joints):
    # Robot base fixed in the Gazebo world
    T = translation(-0.8, 0.9, 0.1)
    frames = {}

    # Joint 1
    T = T @ translation(0, 0, 0.1)
    T = T @ joint_rotation('z', joints[0])
    frames['link1'] = T.copy()

    # Joint 2
    T = T @ translation(0, 0, 0.7) @ rpy(0, pi / 2, 0)
    T = T @ joint_rotation('y', joints[1])
    frames['link2'] = T.copy()

    # Joint 3
    T = T @ translation(0, 0, 0.8)
    T = T @ joint_rotation('y', joints[2])
    frames['link3'] = T.copy()

    # Joint 4
    T = T @ translation(0, 0, 0.7)
    T = T @ joint_rotation('z', joints[3])
    frames['link4'] = T.copy()

    # Joint 5
    T = T @ translation(0, 0, 0.5) @ rpy(0, pi / 2, 0)
    T = T @ joint_rotation('y', joints[4])
    frames['link5'] = T.copy()

    # Joint 6
    T = T @ translation(0, 0, 0.4)
    T = T @ joint_rotation('z', joints[5])
    frames['link6'] = T.copy()

    # Fixed spindle joint: match the URDF origin and rotation
    T_spindle = (
        T
        @ translation(0, 0, 0.10)
        @ rpy(pi, 0, 0)
    )
    
    frames['drill_spindle'] = T_spindle.copy()

    # Drill-bit tip: 0.23 m along spindle local negative Z
    T_tip = T_spindle @ translation(0, 0, -0.23)
    frames['drill_tip'] = T_tip.copy()

    return frames


if __name__ == '__main__':
    # Joint order here is joint1 through joint6.
    home = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    frames = calculate_fk(home)

    print('Robot 2 forward-kinematics check at HOME')
    print('All distances are in metres.')
    print()

    for name, T in frames.items():
        p = T[:3, 3]
        print(
            f'{name:14s}: '
            f'X={p[0]: .3f}, Y={p[1]: .3f}, Z={p[2]: .3f}'
        )

    print()
    print('Expected link6 TF position: approximately (1.200, 0.900, 0.500)')
    print('This script does not send commands to the robot.')
