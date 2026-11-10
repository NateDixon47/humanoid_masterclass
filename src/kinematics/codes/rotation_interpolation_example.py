"""Interpolate between two attitudes through their relative rotation."""

from __future__ import annotations

import numpy as np

from kinematics import rodrigues, rot_to_omega, rpy_to_rot


def interpolate_rotation(
    start: np.ndarray,
    goal: np.ndarray,
    ratio: float,
) -> np.ndarray:
    relative = start.T @ goal
    rotation_vector = rot_to_omega(relative)
    angle = np.linalg.norm(rotation_vector)

    if angle < 1e-12:
        return start.copy()

    axis = rotation_vector / angle
    increment = rodrigues(axis, ratio * angle)
    return start @ increment


def main() -> None:
    start = np.eye(3)
    goal = rpy_to_rot(0.0, np.deg2rad(60.0), np.deg2rad(20.0))

    for ratio in (0.0, 0.5, 1.0):
        attitude = interpolate_rotation(start, goal, ratio)
        print(f"\nInterpolation ratio: {ratio:.1f}")
        print(np.round(attitude, 4))


if __name__ == "__main__":
    main()
