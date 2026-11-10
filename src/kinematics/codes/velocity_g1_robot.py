from __future__ import annotations

import math
import time
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parent
DESCRIPTION_DIR = ROOT / "g1_description"
DEFAULT_URDF = DESCRIPTION_DIR / "g1_29dof.urdf"


def get_joint_index(p, robot_id: int, joint_name: str) -> int:
    for i in range(p.getNumJoints(robot_id)):
        name = p.getJointInfo(robot_id, i)[1].decode("utf-8")
        if name == joint_name:
            return i
    raise ValueError(f"Joint not found: {joint_name}")


def create_debug_items(p) -> dict[str, int]:
    return {
        "r_line": p.addUserDebugLine([0, 0, 0], [0, 0, 0], [0.85, 0, 0], lineWidth=6),
        "omega_line": p.addUserDebugLine([0, 0, 0], [0, 0, 0], [0, 0.25, 1], lineWidth=6),
        "velocity_line": p.addUserDebugLine([0, 0, 0], [0, 0, 0], [0, 0.65, 0], lineWidth=6),
        "r_text": p.addUserDebugText("", [0, 0, 0], [0.85, 0, 0], textSize=1.4),
        "omega_text": p.addUserDebugText("", [0, 0, 0], [0, 0.25, 1], textSize=1.4),
        "velocity_text": p.addUserDebugText("", [0, 0, 0], [0, 0.65, 0], textSize=1.4),
        "panel": p.addUserDebugText("", [0, 0, 0], [0, 0, 0], textSize=1.0),
    }


def format_vector(name: str, vector: np.ndarray) -> str:
    return f"{name} = [{vector[0]: .3f}, {vector[1]: .3f}, {vector[2]: .3f}]"


def update_visualization(
    p,
    item_ids: dict[str, int],
    shoulder_position: tuple[float, float, float],
    hand_position: tuple[float, float, float],
    omega: np.ndarray,
) -> None:
    shoulder = np.array(shoulder_position)
    hand = np.array(hand_position)
    r = shoulder - hand
    velocity = np.cross(omega, r)

    omega_scale = 0.18
    velocity_scale = 0.35
    omega_end = shoulder + omega_scale * omega
    velocity_end = hand + velocity_scale * velocity
    r_mid = 0.5 * (shoulder + hand)
    omega_mid = 0.5 * (shoulder + omega_end)
    velocity_mid = 0.5 * (hand + velocity_end)

    p.addUserDebugLine(
        shoulder,
        hand,
        [0.85, 0, 0],
        lineWidth=6,
        replaceItemUniqueId=item_ids["r_line"],
    )
    p.addUserDebugLine(
        shoulder,
        omega_end,
        [0, 0.25, 1],
        lineWidth=6,
        replaceItemUniqueId=item_ids["omega_line"],
    )
    p.addUserDebugLine(
        hand,
        velocity_end,
        [0, 0.65, 0],
        lineWidth=6,
        replaceItemUniqueId=item_ids["velocity_line"],
    )

    p.addUserDebugText(
        "r = p_k - p",
        r_mid,
        [0.85, 0, 0],
        textSize=1.4,
        replaceItemUniqueId=item_ids["r_text"],
    )
    p.addUserDebugText(
        "omega",
        omega_mid,
        [0, 0.25, 1],
        textSize=1.4,
        replaceItemUniqueId=item_ids["omega_text"],
    )
    p.addUserDebugText(
        "v",
        velocity_mid,
        [0, 0.65, 0],
        textSize=1.4,
        replaceItemUniqueId=item_ids["velocity_text"],
    )

    panel_text = "\n".join(
        [
            format_vector("omega", omega),
            format_vector("p", r),
            format_vector("v", velocity),
        ]
    )
    p.addUserDebugText(
        panel_text,
        [0.75, -0.45, 1.35],
        [0.05, 0.05, 0.05],
        textSize=1.1,
        replaceItemUniqueId=item_ids["panel"],
    )


def main() -> None:
    urdf_path = DEFAULT_URDF.resolve()
    if not urdf_path.exists():
        raise SystemExit(f"URDF file not found: {urdf_path}")

    import pybullet as p
    import pybullet_data

    p.connect(p.GUI, options="--width=900 --height=700")
    p.setAdditionalSearchPath(str(urdf_path.parent))
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
    p.resetDebugVisualizerCamera(
        cameraDistance=2.5,
        cameraYaw=35,
        cameraPitch=-18,
        cameraTargetPosition=[0.0, -0.05, 0.85],
    )

    p.setGravity(0, 0, 0)
    p.loadURDF("plane.urdf")
    robot_id = p.loadURDF(
        str(urdf_path),
        basePosition=[0, 0, 0.85],
        useFixedBase=True,
        flags=p.URDF_USE_INERTIA_FROM_FILE,
    )

    shoulder_joint = get_joint_index(p, robot_id, "right_shoulder_roll_joint")
    elbow_joint = get_joint_index(p, robot_id, "right_elbow_joint")
    hand_link = get_joint_index(p, robot_id, "right_hand_palm_joint")

    initial_elbow = 90.0
    shoulder_angle = math.radians(-35.0)
    shoulder_min = math.radians(-120.0)
    shoulder_max = math.radians(0.0)
    shoulder_speed = math.radians(35.0)

    elbow_slider = p.addUserDebugParameter(
        "right elbow angle (degrees)",
        -60,
        90,
        initial_elbow,
    )

    item_ids = create_debug_items(p)
    previous_time = time.monotonic()

    print("Move the elbow slider and observe how p and v change.")

    try:
        while p.isConnected():
            now = time.monotonic()
            dt = now - previous_time
            previous_time = now

            shoulder_angle += shoulder_speed * dt
            if shoulder_angle > shoulder_max:
                shoulder_angle = shoulder_max
                shoulder_speed *= -1.0
            elif shoulder_angle < shoulder_min:
                shoulder_angle = shoulder_min
                shoulder_speed *= -1.0

            elbow_angle = math.radians(p.readUserDebugParameter(elbow_slider))
            p.resetJointState(robot_id, shoulder_joint, shoulder_angle)
            p.resetJointState(robot_id, elbow_joint, elbow_angle)
            p.stepSimulation()

            shoulder_state = p.getLinkState(
                robot_id,
                shoulder_joint,
                computeForwardKinematics=True,
            )
            hand_state = p.getLinkState(
                robot_id,
                hand_link,
                computeForwardKinematics=True,
            )

            shoulder_rotation = np.array(
                p.getMatrixFromQuaternion(shoulder_state[5])
            ).reshape(3, 3)
            shoulder_axis_world = shoulder_rotation[:, 0]
            omega = shoulder_speed * shoulder_axis_world

            update_visualization(
                p,
                item_ids,
                shoulder_state[4],
                hand_state[4],
                omega,
            )
            time.sleep(1.0 / 240.0)
    except KeyboardInterrupt:
        pass
    finally:
        if p.isConnected():
            p.disconnect()


if __name__ == "__main__":
    main()
