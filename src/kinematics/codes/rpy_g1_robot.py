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


def rot_x(angle: float) -> np.ndarray:
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, c, -s],
            [0.0, s, c],
        ]
    )


def rot_y(angle: float) -> np.ndarray:
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array(
        [
            [c, 0.0, s],
            [0.0, 1.0, 0.0],
            [-s, 0.0, c],
        ]
    )


def rot_z(angle: float) -> np.ndarray:
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array(
        [
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ]
    )


def rpy_to_rot(roll: float, pitch: float, yaw: float) -> np.ndarray:
    # Rotation about the X-axis
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll),  np.cos(roll)]
    ])

    # Rotation about Y-axis
    Ry = np.array([
        [ np.cos(pitch), 0, np.sin(pitch)],
        [0,              1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])

    # Rotation about Z-axis
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw),  np.cos(yaw), 0],
        [0,              0,             1]
    ])

    return Rz @ Ry @ Rx


def create_frame_items(p) -> list[int]:
    item_ids = []
    for _ in range(3):
        item_ids.append(p.addUserDebugLine([0, 0, 0], [0, 0, 0], [1, 1, 1], lineWidth=5))
        item_ids.append(p.addUserDebugText("", [0, 0, 0], [1, 1, 1], textSize=1.2))
    return item_ids


def draw_link_frame(
    p,
    robot_id: int,
    link_id: int,
    item_ids: list[int],
    length: float = 0.35,
) -> None:
    state = p.getLinkState(robot_id, link_id, computeForwardKinematics=True)
    origin = state[4]
    rotation = p.getMatrixFromQuaternion(state[5])

    x_axis = [rotation[0], rotation[3], rotation[6]]
    y_axis = [rotation[1], rotation[4], rotation[7]]
    z_axis = [rotation[2], rotation[5], rotation[8]]

    axes = [
        (x_axis, [1, 0, 0], "x"),
        (y_axis, [0, 0.6, 0], "y"),
        (z_axis, [0, 0.25, 1], "z"),
    ]

    for index, (axis, color, label) in enumerate(axes):
        end = [origin[i] + length * axis[i] for i in range(3)]
        p.addUserDebugLine(
            origin,
            end,
            color,
            lineWidth=5,
            replaceItemUniqueId=item_ids[index * 2],
        )
        p.addUserDebugText(
            label,
            end,
            color,
            textSize=1.4,
            replaceItemUniqueId=item_ids[index * 2 + 1],
        )


def create_matrix_panel(p) -> int:
    return p.addUserDebugText("", [0, 0, 0], [0, 0, 0], textSize=1.0)


def matrix_text(roll: float, pitch: float, yaw: float, rotation: np.ndarray) -> str:
    degrees = np.rad2deg([roll, pitch, yaw])
    rows = [
        "[" + " ".join(f"{value: .3f}" for value in row) + "]"
        for row in rotation
    ]
    return "\n".join(
        [
            "Waist RPY rotation",
            f"roll  = {degrees[0]: .1f} deg",
            f"pitch = {degrees[1]: .1f} deg",
            f"yaw   = {degrees[2]: .1f} deg",
            "R = Rz(yaw) Ry(pitch) Rx(roll)",
            rows[0],
            rows[1],
            rows[2],
        ]
    )


def update_matrix_panel(
    p,
    panel_id: int,
    roll: float,
    pitch: float,
    yaw: float,
    rotation: np.ndarray,
) -> None:
    p.addUserDebugText(
        matrix_text(roll, pitch, yaw, rotation),
        [0.85, -0.45, 1.35],
        [0.05, 0.05, 0.05],
        textSize=1.15,
        replaceItemUniqueId=panel_id,
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
        cameraDistance=2.6,
        cameraYaw=35,
        cameraPitch=-18,
        cameraTargetPosition=[0.0, 0.0, 0.95],
    )

    p.setGravity(0, 0, 0)
    p.loadURDF("plane.urdf")
    robot_id = p.loadURDF(
        str(urdf_path),
        basePosition=[0, 0, 0.85],
        useFixedBase=True,
        flags=p.URDF_USE_INERTIA_FROM_FILE,
    )

    waist_yaw = get_joint_index(p, robot_id, "waist_yaw_joint")
    waist_roll = get_joint_index(p, robot_id, "waist_roll_joint")
    waist_pitch = get_joint_index(p, robot_id, "waist_pitch_joint")

    roll_slider = p.addUserDebugParameter("waist roll phi (degrees)", -30, 30, 0)
    pitch_slider = p.addUserDebugParameter("waist pitch theta (degrees)", -30, 30, 0)
    yaw_slider = p.addUserDebugParameter("waist yaw psi (degrees)", -45, 45, 0)

    torso_frame = create_frame_items(p)
    panel_id = create_matrix_panel(p)

    print("Move the waist roll, pitch, and yaw sliders to update the rotation matrix.")

    try:
        while p.isConnected():
            roll = math.radians(p.readUserDebugParameter(roll_slider))
            pitch = math.radians(p.readUserDebugParameter(pitch_slider))
            yaw = math.radians(p.readUserDebugParameter(yaw_slider))

            p.resetJointState(robot_id, waist_yaw, yaw)
            p.resetJointState(robot_id, waist_roll, roll)
            p.resetJointState(robot_id, waist_pitch, pitch)
            p.stepSimulation()

            rotation = rpy_to_rot(roll, pitch, yaw)
            draw_link_frame(p, robot_id, waist_pitch, torso_frame)
            update_matrix_panel(p, panel_id, roll, pitch, yaw, rotation)
            time.sleep(1.0 / 240.0)
    except KeyboardInterrupt:
        pass
    finally:
        if p.isConnected():
            p.disconnect()


if __name__ == "__main__":
    main()
