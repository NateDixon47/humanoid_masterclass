"""Compute the velocity of one point on a moving rigid body."""

from __future__ import annotations

import numpy as np


def main() -> None:
    reference_point = np.array([0.0, 0.0, 0.7])
    observed_point = np.array([0.2, 0.0, 0.7])
    linear_velocity = np.array([0.0, 0.0, 0.1])
    angular_velocity = np.array([0.0, 0.0, 2.0])

    point_velocity = linear_velocity + np.cross(
        angular_velocity,
        observed_point - reference_point,
    )
    print("Velocity of the point:", point_velocity)


if __name__ == "__main__":
    main()
