"""Transform a point from a local frame to the world frame."""

from __future__ import annotations

import numpy as np


def main() -> None:
    phi = np.deg2rad(30.0)
    rotation = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, np.cos(phi), -np.sin(phi)],
            [0.0, np.sin(phi), np.cos(phi)],
        ]
    )
    frame_position = np.array([0.2, 0.1, 1.0])
    local_point = np.array([0.0, 0.0, -0.4])

    world_point = frame_position + rotation @ local_point
    print(np.round(world_point, 4))


if __name__ == "__main__":
    main()
