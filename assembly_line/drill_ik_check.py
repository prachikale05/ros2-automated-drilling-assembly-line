#!/usr/bin/env python3

import numpy as np
from scipy.optimize import least_squares

from .drill_geometry_check import calculate_fk


# ============================================================
# ROBOT 2 JOINT LIMITS (RADIANS)
# ============================================================

LOWER = np.array([
    -3.14, -2.5, -2.5, -3.14, -2.5, -3.14
])

UPPER = np.array([
     3.14,  2.5,  2.5,  3.14,  2.5,  3.14
])


# ============================================================
# WORKPIECE GEOMETRY (WORLD COORDINATES)
# ============================================================

WORKPIECE_X = -0.1375
WORKPIECE_Y = 0.0

# Model entity Z = 0.87 m
# Link offset Z = 0.15 m
# Cube centre Z = 1.02 m
# Cube top surface Z = 1.17 m

TOP_Z = 1.170

APPROACH_HEIGHT = 0.100
DRILL_DEPTH = 0.150


# Exactly two diagonal holes

HOLES = [
    (-0.080,  0.080),
    ( 0.080, -0.080),
]


# ============================================================
# FORWARD KINEMATICS: DRILL TIP AND DRILL AXIS
# ============================================================

def fk_tip_and_axis(joints):

    frames = calculate_fk(joints)

    tip = frames["drill_tip"][:3, 3]

    spindle_rotation = frames["drill_spindle"][:3, :3]

    # Drill bit extends along spindle local negative Z.

    drill_axis = spindle_rotation @ np.array([
        0.0, 0.0, -1.0
    ])

    return tip, drill_axis


# ============================================================
# INVERSE KINEMATICS
# ============================================================

def solve_target(target, initial_guess):

    target = np.asarray(target, dtype=float)

    safe_lower = LOWER + 0.05
    safe_upper = UPPER - 0.05

    initial_guess = np.clip(
        initial_guess,
        safe_lower,
        safe_upper
    )

    # Multiple starting configurations help find solutions.

    starts = [
        initial_guess,

        np.zeros(6),

        np.array([
            0.0, -1.0, 1.5, 0.0, 0.5, 0.0
        ]),

        np.array([
            0.0, -1.5, 2.0, 0.5, 0.5, 0.0
        ]),

        np.array([
            0.0, -0.5, 1.0, -0.5, 1.0, 0.0
        ]),

        np.array([
            2.18, -1.93, -1.88, 0.0, 0.67, 0.23
        ]),
    ]

    candidates = []

    for start in starts:

        start = np.clip(
            start,
            safe_lower,
            safe_upper
        )

        def residual(q):

            tip, axis = fk_tip_and_axis(q)

            position_error = (tip - target) * 10.0

            desired_axis = np.array([
                0.0, 0.0, -1.0
            ])

            orientation_error = (
                axis - desired_axis
            ) * 2.0

            # Small regularization favours a nearby joint solution.

            joint_error = (q - initial_guess) * 0.002

            return np.concatenate([
                position_error,
                orientation_error,
                joint_error
            ])

        result = least_squares(
            residual,
            start,
            bounds=(safe_lower, safe_upper),
            max_nfev=4000,
            ftol=1e-10,
            xtol=1e-10,
            gtol=1e-10
        )

        tip, axis = fk_tip_and_axis(result.x)

        position_error = np.linalg.norm(
            tip - target
        )

        orientation_error = np.linalg.norm(
            axis - np.array([0.0, 0.0, -1.0])
        )

        valid = (
            result.success
            and position_error < 0.003
            and orientation_error < 0.02
        )

        if valid:

            joint_distance = np.linalg.norm(
                result.x - initial_guess
            )

            candidates.append((
                joint_distance,
                result.x,
                tip,
                axis,
                position_error,
                orientation_error
            ))

    if not candidates:

        print("WARNING: No valid IK solution found.")

        return None

    # Prefer the valid solution closest to the previous pose.

    candidates.sort(key=lambda item: item[0])

    _, q, tip, axis, pos_error, ori_error = candidates[0]

    return q, tip, axis, pos_error, ori_error


# ============================================================
# DISPLAY RESULTS
# ============================================================

def display_result(label, target, solution):

    print()
    print("--------------------------------------------")
    print(label)
    print("--------------------------------------------")

    print("Target tip position:", np.round(target, 4))

    if solution is None:

        print("RESULT: FAILED")
        return None

    q, tip, axis, pos_error, ori_error = solution

    print("Joint angles (radians):")
    print(np.round(q, 4).tolist())

    print("Joint angles (degrees):")
    print(np.round(np.degrees(q), 2).tolist())

    print("Calculated drill-tip position:")
    print(np.round(tip, 4))

    print("Drill direction:")
    print(np.round(axis, 5))

    print(f"Position error: {pos_error:.6f} m")

    print(f"Orientation error: {ori_error:.6f}")

    print("RESULT: PASS")

    return q


# ============================================================
# MAIN CALCULATION
# ============================================================

def main():

    print()
    print("============================================")
    print(" ROBOT 2 - TWO HOLE DRILLING IK CHECK")
    print("============================================")

    print("Calculation only. No robot commands are sent.")

    print()
    print("Workpiece centre:")
    print(
        WORKPIECE_X,
        WORKPIECE_Y,
        TOP_Z - 0.150
    )

    print(f"Top surface Z: {TOP_Z:.3f} m")

    print(f"Approach height: {APPROACH_HEIGHT:.3f} m")

    print(f"Drilling depth: {DRILL_DEPTH:.3f} m")

    print("Hole diameter: 0.025 m")

    print("Number of holes: 2")

    print()

    current_guess = np.zeros(6)

    all_passed = True

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

        print()
        print("============================================")
        print(f"HOLE {index}")
        print("============================================")

        print(f"Hole centre: X={x:.4f}, Y={y:.4f}")

        # ----------------------------------------
        # APPROACH
        # ----------------------------------------

        approach_solution = solve_target(
            approach_target,
            current_guess
        )

        approach_q = display_result(
            f"HOLE {index} - APPROACH",
            approach_target,
            approach_solution
        )

        if approach_q is None:

            all_passed = False
            print("Skipping drilling target.")
            continue

        # ----------------------------------------
        # DRILLING
        # ----------------------------------------

        drilling_solution = solve_target(
            drilling_target,
            approach_q
        )

        drilling_q = display_result(
            f"HOLE {index} - DRILLING POSITION",
            drilling_target,
            drilling_solution
        )

        if drilling_q is None:

            all_passed = False
            continue

        # Next hole starts from the previous
        # hole's approach configuration.

        current_guess = approach_q.copy()

    # ----------------------------------------
    # FINAL STATUS
    # ----------------------------------------

    print()
    print("============================================")
    print("FINAL IK VALIDATION")
    print("============================================")

    if all_passed:

        print("Both holes have valid approach and")
        print("drilling target solutions.")

        print("IK VALIDATION: PASS")

    else:

        print("One or more target positions failed.")

        print("IK VALIDATION: FAIL")

    print()
    print("IMPORTANT:")
    print("This checks mathematical IK only.")
    print("It does not check collision-free paths.")
    print("Do not run the drilling controller yet.")


if __name__ == "__main__":
    main()
