#!/usr/bin/env python3

import numpy as np

from drill_geometry_check import calculate_fk
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


# Workpiece dimensions
HALF_SIZE = 0.150

BOTTOM_Z = TOP_Z - 0.300
HOLE_RADIUS = 0.0125

# Current drill-bit radius from the Xacro
DRILL_RADIUS = 0.010

# Available radial clearance inside each hole
RADIAL_CLEARANCE = HOLE_RADIUS - DRILL_RADIUS

# Sampling resolution
SAMPLES = 101


def tool_points(q):
    """Return sampled points along the drill tool."""

    frames = calculate_fk(q)

    tip = frames["drill_tip"][:3, 3]
    spindle = frames["drill_spindle"][:3, 3]

    points = []

    for t in np.linspace(0.0, 1.0, 12):
        points.append(tip + t * (spindle - tip))

    return np.array(points)


def inside_workpiece(point):

    x, y, z = point

    inside_xy = (
        abs(x - WORKPIECE_X) <= HALF_SIZE + DRILL_RADIUS
        and abs(y - WORKPIECE_Y) <= HALF_SIZE + DRILL_RADIUS
    )

    inside_z = BOTTOM_Z <= z <= TOP_Z

    return inside_xy and inside_z


def check_segment(label, q_start, q_end, allowed_hole=None):

    print()
    print("============================================")
    print(label)
    print("============================================")

    collisions = 0
    maximum_lateral_error = 0.0
    maximum_orientation_error = 0.0

    for t in np.linspace(0.0, 1.0, SAMPLES):

        q = q_start + t * (q_end - q_start)

        tip, axis = fk_tip_and_axis(q)

        points = tool_points(q)

        if allowed_hole is not None:

            dx, dy = allowed_hole

            hole_x = WORKPIECE_X + dx
            hole_y = WORKPIECE_Y + dy

            lateral_error = np.linalg.norm(
                tip[:2] - np.array([hole_x, hole_y])
            )

            orientation_error = np.linalg.norm(
                axis - np.array([0.0, 0.0, -1.0])
            )

            maximum_lateral_error = max(
                maximum_lateral_error,
                lateral_error
            )

            maximum_orientation_error = max(
                maximum_orientation_error,
                orientation_error
            )

            # Check that the drill stays within the hole
            # while passing through the workpiece.

            for point in points:

                if inside_workpiece(point):

                    point_error = np.linalg.norm(
                        point[:2] -
                        np.array([hole_x, hole_y])
                    )

                    if point_error > RADIAL_CLEARANCE:
                        collisions += 1
                        break

            if (
                lateral_error > RADIAL_CLEARANCE
                or orientation_error > 0.02
            ):
                collisions += 1

        else:

            # During travel, the drill must not intersect
            # the solid workpiece.

            if any(inside_workpiece(point) for point in points):
                collisions += 1

    if collisions == 0:

        print("PATH CHECK: PASS")

    else:

        print("PATH CHECK: FAIL")
        print("Collision/clearance violations:", collisions)

    if allowed_hole is not None:

        print(
            "Maximum lateral deviation:",
            f"{maximum_lateral_error:.6f} m"
        )

        print(
            "Maximum orientation deviation:",
            f"{maximum_orientation_error:.6f}"
        )

    return collisions == 0


def main():

    print()
    print("============================================")
    print(" ROBOT 2 - DRILLING PATH VALIDATION")
    print("============================================")

    print("Calculation only. No robot commands are sent.")

    print("Workpiece top:", TOP_Z)
    print("Workpiece bottom:", BOTTOM_Z)

    print("Drill radius:", DRILL_RADIUS)
    print("Hole radius:", HOLE_RADIUS)

    print("Radial clearance:", RADIAL_CLEARANCE)

    print("Samples per movement:", SAMPLES)

    home = np.zeros(6)

    solutions = []

    current_guess = home.copy()

    # Calculate approach and drilling configurations.

    for index, (dx, dy) in enumerate(HOLES, start=1):

        x = WORKPIECE_X + dx
        y = WORKPIECE_Y + dy

        approach_target = np.array([
            x,
            y,
            TOP_Z + APPROACH_HEIGHT
        ])

        drilling_target = np.array([
            x,
            y,
            TOP_Z - DRILL_DEPTH
        ])

        approach_solution = solve_target(
            approach_target,
            current_guess
        )

        if approach_solution is None:
            print(f"HOLE {index} APPROACH IK FAILED")
            return

        approach_q = approach_solution[0]

        drilling_solution = solve_target(
            drilling_target,
            approach_q
        )

        if drilling_solution is None:
            print(f"HOLE {index} DRILLING IK FAILED")
            return

        drilling_q = drilling_solution[0]

        solutions.append((approach_q, drilling_q))

        current_guess = approach_q.copy()

    h1_approach, h1_drill = solutions[0]
    h2_approach, h2_drill = solutions[1]

    results = []

    # HOME to first hole

    results.append(check_segment(
        "1. HOME -> HOLE 1 APPROACH",
        home,
        h1_approach
    ))

    # First hole drilling and retraction

    results.append(check_segment(
        "2. HOLE 1 APPROACH -> DRILL",
        h1_approach,
        h1_drill,
        HOLES[0]
    ))

    results.append(check_segment(
        "3. HOLE 1 DRILL -> RETRACT",
        h1_drill,
        h1_approach,
        HOLES[0]
    ))

    # Transfer between holes

    results.append(check_segment(
        "4. HOLE 1 -> HOLE 2 TRANSFER",
        h1_approach,
        h2_approach
    ))

    # Second hole drilling and retraction

    results.append(check_segment(
        "5. HOLE 2 APPROACH -> DRILL",
        h2_approach,
        h2_drill,
        HOLES[1]
    ))

    results.append(check_segment(
        "6. HOLE 2 DRILL -> RETRACT",
        h2_drill,
        h2_approach,
        HOLES[1]
    ))

    # Return HOME

    results.append(check_segment(
        "7. HOLE 2 -> HOME",
        h2_approach,
        home
    ))

    print()
    print("============================================")
    print(" FINAL PATH VALIDATION")
    print("============================================")

    if all(results):

        print("ALL SEVEN PATH CHECKS PASSED.")

        print("Drill-tool trajectory validation: PASS")

    else:

        print("ONE OR MORE PATH CHECKS FAILED.")

        print("Do not run the drilling controller.")

    print()
    print("NOTE:")
    print("Only the drill-tool trajectory was checked.")
    print("Robot arm link collisions are not validated.")
    print("No commands were sent to Gazebo.")


if __name__ == "__main__":
    main()
