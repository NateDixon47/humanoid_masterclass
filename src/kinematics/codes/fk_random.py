"""Generate random toy biped poses using forward kinematics."""

from __future__ import annotations

import argparse

import matplotlib.pyplot as plt
import numpy as np

from robot_model import setup_biped_robot, set_random_leg_angles
from visualization import draw_all_joints, set_biped_axes


def draw_random_grid(seed: int = 0, rows: int = 3, cols: int = 3) -> None:
    rng = np.random.default_rng(seed)
    links = setup_biped_robot()

    figure = plt.figure(figsize=(10, 10))
    for index in range(rows * cols):
        set_random_leg_angles(links, rng)
        ax = figure.add_subplot(rows, cols, index + 1, projection="3d")
        draw_all_joints(ax, links, 1)
        set_biped_axes(ax, show_labels=False)
        ax.set_xticklabels([])
        ax.set_yticklabels([])
        ax.set_zticklabels([])

    figure.subplots_adjust(left=0.02, right=0.98, bottom=0.02, top=0.98, wspace=0.04, hspace=0.04)
    plt.show()


def draw_single_pose(seed: int = 0) -> None:
    rng = np.random.default_rng(seed)
    links = setup_biped_robot()
    set_random_leg_angles(links, rng)

    figure = plt.figure(figsize=(6, 6))
    ax = figure.add_subplot(111, projection="3d")
    draw_all_joints(ax, links, 1)
    set_biped_axes(ax)
    figure.tight_layout()
    plt.show()


def main() -> None:
    parser = argparse.ArgumentParser(description="Draw random toy biped poses.")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--single", action="store_true", help="Draw a single pose instead of a 3x3 grid.")
    args = parser.parse_args()

    if args.single:
        draw_single_pose(args.seed)
    else:
        draw_random_grid(args.seed)


if __name__ == "__main__":
    main()
