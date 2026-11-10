"""Test Rodrigues' formula with a simple numeric rotation."""

from __future__ import annotations

import numpy as np

from kinematics import rodrigues, rot_to_omega


def main() -> None:
    axis = np.array([0.0, 0.0, 1.0])
    angle = np.deg2rad(20.0)
    rotation = rodrigues(axis, angle)

    print(np.round(rotation, 4))
    print("rotation vector:", np.round(rot_to_omega(rotation), 4))


if __name__ == "__main__":
    main()
