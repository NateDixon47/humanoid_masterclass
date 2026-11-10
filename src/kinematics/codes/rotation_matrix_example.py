"""Build a rotation matrix from roll, pitch, and yaw angles."""

from __future__ import annotations

import numpy as np

from kinematics import rpy_to_rot


def main() -> None:
    roll, pitch, yaw = np.deg2rad([10.0, 20.0, 30.0])
    rotation = rpy_to_rot(roll, pitch, yaw)

    local_point = np.array([0.30, 0.0, 0.0])
    rotated_point = rotation @ local_point

    print("RPY rotation matrix:")
    print(np.round(rotation, 4))
    print("Rotated point:", np.round(rotated_point, 4))


if __name__ == "__main__":
    main()
