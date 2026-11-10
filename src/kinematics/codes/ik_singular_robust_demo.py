"""Visualize robust inverse kinematics as a leg approaches singularity."""

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


def weighted_cost(error: np.ndarray, weight: np.ndarray) -> float:
    """Return the weighted squared pose error."""
    return float(error.T @ weight @ error)


def levenberg_marquardt_step(
    links: list,
    route: list[int],
    target: Pose,
    max_iterations: int = 10,
) -> float:
    """Apply a damped Levenberg-Marquardt IK update."""
    position_weight = 1.0 / 0.3
    attitude_weight = 1.0 / (2.0 * np.pi)
    error_weight = np.diag(
        [
            position_weight,
            position_weight,
            position_weight,
            attitude_weight,
            attitude_weight,
            attitude_weight,
        ]
    )
    joint_weight = np.eye(len(route))

    forward_kinematics(links, BODY)
    error = calc_pose_error(target, links[route[-1]])
    cost = weighted_cost(error, error_weight)

    for _ in range(max_iterations):
        if cost < 1e-12:
            break

        jacobian = calc_jacobian(links, route)
        hessian = jacobian.T @ error_weight @ jacobian
        damping = joint_weight * (cost + 0.002)
        gradient = jacobian.T @ error_weight @ error
        delta_q = np.linalg.solve(hessian + damping, gradient)

        move_joints(links, route, delta_q)
        forward_kinematics(links, BODY)

        new_error = calc_pose_error(target, links[route[-1]])
        new_cost = weighted_cost(new_error, error_weight)
        if new_cost < cost:
            error = new_error
            cost = new_cost
        else:
            move_joints(links, route, -delta_q)
            forward_kinematics(links, BODY)
            break

    return float(np.linalg.norm(error))


def setup_posture_copy(angles: np.ndarray) -> list:
    """Build an independent robot model for one recorded posture."""
    links = setup_biped_robot()
    route = find_route(links, RLEG_J5)
    set_route_angles(links, route, angles)
    return links


def run_experiment() -> tuple[np.ndarray, np.ndarray, list[list], float]:
    """Move the right foot forward and record robust IK results."""
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
        levenberg_marquardt_step(links, route, target)
        joint_angles[step] = [links[link_id].q for link_id in route]

        if step % 10 == 0:
            sampled_postures.append(setup_posture_copy(joint_angles[step]))

    return x_targets, joint_angles, sampled_postures, singular_point


def create_figure() -> plt.Figure:
    """Create the overlapping-posture view and robust joint-angle graph."""
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
    graph_axis.plot(x_targets, TO_DEG * joint_angles[:, 2], color="#333399", linewidth=2.0, label="RLEG_J2")
    graph_axis.plot(x_targets, TO_DEG * joint_angles[:, 3], color="#00a651", linewidth=2.0, label="RLEG_J3")
    graph_axis.plot(x_targets, TO_DEG * joint_angles[:, 4], color="#e52b2b", linewidth=2.0, label="RLEG_J4")
    graph_axis.axvline(singular_point, color="magenta", linestyle="-.", linewidth=1.0)
    graph_axis.set_xlim(0.0, 0.4)
    graph_axis.set_ylim(-40.0, 60.0)
    graph_axis.set_xlabel("x [m]")
    graph_axis.set_ylabel("q [deg]")
    graph_axis.grid(True, alpha=0.3)
    graph_axis.legend(loc="center left", bbox_to_anchor=(0.06, 0.63))
    graph_axis.annotate(
        "",
        xy=(0.0, 57.0),
        xytext=(singular_point, 57.0),
        arrowprops={"arrowstyle": "<->", "color": "black"},
    )
    graph_axis.text(singular_point * 0.5, 53.5, "Non singular", ha="center")
    graph_axis.annotate(
        "",
        xy=(singular_point, 57.0),
        xytext=(0.4, 57.0),
        arrowprops={"arrowstyle": "<->", "color": "black"},
    )
    graph_axis.text((singular_point + 0.4) * 0.5, 53.5, "Singular", ha="center")
    figure.tight_layout()

    return figure


def main() -> None:
    create_figure()
    plt.show()


if __name__ == "__main__":
    main()
