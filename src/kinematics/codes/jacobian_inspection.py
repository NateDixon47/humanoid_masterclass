"""Compare non-singular and singular Jacobian postures of the toy biped."""

from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np

from inverse_kinematics import calc_jacobian, find_route
from kinematics import forward_kinematics
from robot_model import BODY, RLEG_J5, setup_biped_robot
from visualization import draw_all_joints, set_biped_axes


def set_right_leg_angles(links: list, route: list[int], angles: np.ndarray) -> None:
    """Assign joint angles to the right-leg route and update forward kinematics."""
    for link_id, angle in zip(route, angles):
        links[link_id].q = float(angle)
    forward_kinematics(links, BODY)


def joint_speed_report(label: str, links: list, route: list[int]) -> None:
    """Print Jacobian quality and the joint speed for a vertical foot command."""
    jacobian = calc_jacobian(links, route)
    singular_values = np.linalg.svd(jacobian, compute_uv=False)
    rank = np.linalg.matrix_rank(jacobian)
    foot_velocity = np.array([0.0, 0.0, 0.1, 0.0, 0.0, 0.0])

    print(f"\n{label}")
    print("route:", " -> ".join(links[link_id].name for link_id in route))
    print("singular values:", np.round(singular_values, 6))
    print("rank:", rank)
    try:
        joint_speed = np.linalg.solve(jacobian, foot_velocity)
        print("joint speeds for vertical foot velocity:")
        print(np.round(joint_speed, 4))
    except np.linalg.LinAlgError:
        print("joint speeds for vertical foot velocity:")
        print("singular matrix: no reliable inverse exists")


def draw_posture_pair() -> None:
    """Display a side-by-side image of a non-singular and singular posture."""
    route_angles = [
        ("(a) non-singular", np.deg2rad([0.0, 0.0, -5.0, 10.0, -5.0, 0.0])),
        ("(b) singular", np.zeros(6)),
    ]

    figure = plt.figure(figsize=(8, 4.5))
    for index, (title, angles) in enumerate(route_angles, start=1):
        links = setup_biped_robot()
        route = find_route(links, RLEG_J5)
        set_right_leg_angles(links, route, angles)

        ax = figure.add_subplot(1, 2, index, projection="3d")
        draw_all_joints(ax, links, BODY)
        set_biped_axes(ax, show_labels=False)
        ax.set_xticklabels([])
        ax.set_yticklabels([])
        ax.set_zticklabels([])
        ax.set_title(title, y=-0.10)

    figure.tight_layout()
    plt.show()


def main() -> None:
    route_angles = [
        ("Non-singular bent-knee posture", np.deg2rad([0.0, 0.0, -5.0, 10.0, -5.0, 0.0])),
        ("Singular straight-leg posture", np.zeros(6)),
    ]

    for label, angles in route_angles:
        links = setup_biped_robot()
        route = find_route(links, RLEG_J5)
        set_right_leg_angles(links, route, angles)
        joint_speed_report(label, links, route)

    draw_posture_pair()


if __name__ == "__main__":
    main()
