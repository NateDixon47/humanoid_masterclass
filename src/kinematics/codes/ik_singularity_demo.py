"""Visualize numerical inverse kinematics as a leg approaches singularity."""

from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np

from inverse_kinematics import Pose, calc_jacobian, calc_pose_error, find_route, move_joints
from kinematics import forward_kinematics
from robot_model import BODY, RLEG_J5, setup_biped_robot
from visualization import draw_all_joints


TO_RAD = np.pi / 180.0
TO_DEG = 180.0 / np.pi


def set_route_angles(links: list, route: list[int], angles: np.ndarray) -> None:
    """Assign joint angles along one kinematic route."""
    for link_id, angle in zip(route, angles):
        links[link_id].q = float(angle)
    forward_kinematics(links, BODY)


def newton_raphson_step(
    links: list,
    route: list[int],
    target: Pose,
    gain: float = 0.9,
    max_iterations: int = 10,
) -> float:
    """Apply the undamped Newton-Raphson IK update used in the basic solver."""
    forward_kinematics(links, BODY)
    error = calc_pose_error(target, links[route[-1]])

    for _ in range(max_iterations):
        if np.linalg.norm(error) < 1e-6:
            break
        jacobian = calc_jacobian(links, route)
        try:
            delta_q = gain * np.linalg.solve(jacobian, error)
        except np.linalg.LinAlgError:
            delta_q = gain * np.linalg.lstsq(jacobian, error, rcond=None)[0]
        move_joints(links, route, delta_q)
        forward_kinematics(links, BODY)
        error = calc_pose_error(target, links[route[-1]])

    return float(np.linalg.norm(error))


def run_experiment() -> tuple[np.ndarray, np.ndarray, list[list], float]:
    """Move the right foot forward and record angles and sampled postures."""
    links = setup_biped_robot()
    route = find_route(links, RLEG_J5)
    set_route_angles(
        links,
        route,
        np.array([0.0, 0.0, -25.0, 50.0, -25.0, 0.0]) * TO_RAD,
    )

    target = Pose(p=links[RLEG_J5].p.copy(), R=links[RLEG_J5].R.copy())
    hip_to_foot_height = np.linalg.norm(links[route[0]].p - links[RLEG_J5].p)
    singular_point = np.sqrt(0.6**2 - hip_to_foot_height**2)
    x_targets = np.arange(0.0, 0.4001, 0.005)
    joint_angles = np.zeros((len(x_targets), len(route)))
    sampled_postures: list[list] = []

    for step, x_target in enumerate(x_targets):
        target.p[0] = x_target
        newton_raphson_step(links, route, target)
        joint_angles[step] = [links[link_id].q for link_id in route]

        if step % 10 == 0:
            sampled_postures.append(setup_posture_copy(joint_angles[step]))

    return x_targets, joint_angles, sampled_postures, singular_point


def setup_posture_copy(angles: np.ndarray) -> list:
    """Build an independent robot model for one recorded posture."""
    links = setup_biped_robot()
    route = find_route(links, RLEG_J5)
    set_route_angles(links, route, angles)
    return links


def create_figure() -> plt.Figure:
    """Create the overlapping-posture view and joint-angle graph."""
    x_targets, joint_angles, sampled_postures, singular_point = run_experiment()

    figure = plt.figure(figsize=(12.0, 5.2))
    posture_axis = figure.add_subplot(1, 2, 1, projection="3d")
    for links in sampled_postures:
        draw_all_joints(posture_axis, links, BODY)
    posture_axis.set_xlim(-0.1, 0.5)
    posture_axis.set_ylim(-0.25, 0.25)
    posture_axis.set_zlim(0.0, 1.3)
    posture_axis.set_box_aspect((0.6, 0.5, 1.3))
    posture_axis.view_init(elev=13, azim=46, roll=0)
    posture_axis.set_xlabel("x [m]")
    posture_axis.set_ylabel("y [m]")
    posture_axis.set_zlabel("z [m]")
    posture_axis.grid(True)
    graph_axis = figure.add_subplot(1, 2, 2)
    graph_axis.plot(x_targets, TO_DEG * joint_angles[:, 2], linewidth=2.0, label="RLEG_J2")
    graph_axis.plot(x_targets, TO_DEG * joint_angles[:, 3], linewidth=2.0, label="RLEG_J3")
    graph_axis.plot(x_targets, TO_DEG * joint_angles[:, 4], linewidth=2.0, label="RLEG_J4")
    graph_axis.axvline(singular_point, color="magenta", linestyle="-.", linewidth=1.0)
    graph_axis.set_xlim(0.0, 0.4)
    graph_axis.set_ylim(-40.0, 60.0)
    graph_axis.set_xlabel("x [m]")
    graph_axis.set_ylabel("q [deg]")
    graph_axis.grid(True, alpha=0.3)
    graph_axis.legend()
    figure.tight_layout()

    return figure


def main() -> None:
    create_figure()
    plt.show()


if __name__ == "__main__":
    main()
