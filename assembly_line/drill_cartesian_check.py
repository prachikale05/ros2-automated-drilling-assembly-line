import numpy as np

from drill_ik_check import (
    solve_target,
    fk_tip_and_axis,
    WORKPIECE_X,
    WORKPIECE_Y,
    TOP_Z,
    APPROACH_HEIGHT,
    DRILL_DEPTH,
    HOLES,
)

HOLE_RADIUS = 0.0125
DRILL_RADIUS = 0.010
RADIAL_CLEARANCE = HOLE_RADIUS - DRILL_RADIUS

STEP = 0.005  # 5 mm Cartesian increments


def validate_hole(index, offset, initial_q):

    dx, dy = offset

    hole_x = WORKPIECE_X + dx
    hole_y = WORKPIECE_Y + dy

    approach_z = TOP_Z + APPROACH_HEIGHT
    drill_z = TOP_Z - DRILL_DEPTH

    heights = np.arange(
        approach_z,
        drill_z - 0.0001,
        -STEP
    )

    previous_q = initial_q.copy()

    max_lateral_error = 0.0
    max_position_error = 0.0
    max_orientation_error = 0.0
    max_joint_step = 0.0

    print()
    print("======================================")
    print(f"HOLE {index}: CARTESIAN PATH")
    print("======================================")

    for z in heights:

        target = np.array([hole_x, hole_y, z])

        solution = solve_target(target, previous_q)

        if solution is None:
            print(f"IK FAILED at z={z:.4f}")
            return None

        q, tip, axis, pos_error, ori_error = solution

        lateral_error = np.linalg.norm(
            tip[:2] - np.array([hole_x, hole_y])
        )

        joint_step = np.max(np.abs(q - previous_q))

        max_lateral_error = max(
            max_lateral_error, lateral_error
        )

        max_position_error = max(
            max_position_error, pos_error
        )

        max_orientation_error = max(
            max_orientation_error, ori_error
        )

        max_joint_step = max(
            max_joint_step, joint_step
        )

        # Strict checks for this drilling path.

        if pos_error > 0.0005:
            print(f"Position error too high at z={z:.4f}")
            return None

        if lateral_error > RADIAL_CLEARANCE:
            print(f"Drill clearance failure at z={z:.4f}")
            return None

        if ori_error > 0.002:
            print(f"Drill orientation failure at z={z:.4f}")
            return None

        if joint_step > 0.15:
            print(f"Large joint change at z={z:.4f}")
            return None

        previous_q = q.copy()

    print("Cartesian IK: PASS")
    print(f"Maximum lateral error: {max_lateral_error:.6f} m")
    print(f"Maximum position error: {max_position_error:.6f} m")
    print(f"Maximum orientation error: {max_orientation_error:.6f}")
    print(f"Maximum joint step: {max_joint_step:.6f} rad")

    return previous_q


def main():

    print("ROBOT 2 CARTESIAN DRILLING VALIDATION")
    print("No robot movement commands will be sent.")

    # Use the same HOME configuration as the previous validation.
    home = np.zeros(6)

    # First, calculate the approach configuration for Hole 1.
    dx, dy = HOLES[0]

    first_target = np.array([
        WORKPIECE_X + dx,
        WORKPIECE_Y + dy,
        TOP_Z + APPROACH_HEIGHT
    ])

    first_solution = solve_target(first_target, home)

    if first_solution is None:
        print("Hole 1 approach IK failed.")
        return

    q_h1_approach = first_solution[0]

    # Validate the complete Cartesian drilling path for Hole 1.
    q_h1_drill = validate_hole(
        1,
        HOLES[0],
        q_h1_approach
    )

    if q_h1_drill is None:
        print("HOLE 1 PATH FAILED.")
        return

    # Retract at Hole 1 using the same validated approach configuration.
    # Then calculate Hole 2 approach from that configuration.
    dx, dy = HOLES[1]

    second_target = np.array([
        WORKPIECE_X + dx,
        WORKPIECE_Y + dy,
        TOP_Z + APPROACH_HEIGHT
    ])

    second_solution = solve_target(
        second_target,
        q_h1_approach
    )

    if second_solution is None:
        print("Hole 2 approach IK failed.")
        return

    q_h2_approach = second_solution[0]

    # Validate the complete Cartesian drilling path for Hole 2.
    q_h2_drill = validate_hole(
        2,
        HOLES[1],
        q_h2_approach
    )

    if q_h2_drill is None:
        print("HOLE 2 PATH FAILED.")
        return

    print()
    print("======================================")
    print("FINAL RESULT")
    print("======================================")
    print("Both Cartesian drilling paths passed.")
    print("This is still a mathematical validation only.")
    print("No Gazebo movement commands were sent.")


if __name__ == "__main__":
    main()
