from __future__ import annotations

import math
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parent
DESCRIPTION_DIR = ROOT / "g1_description"
DEFAULT_URDF = DESCRIPTION_DIR / "g1_29dof.urdf"
USE_FIXED_BASE = True


def get_joint_index(p, robot_id: int, joint_name: str) -> int:
    for i in range(p.getNumJoints(robot_id)):
        name = p.getJointInfo(robot_id, i)[1].decode("utf-8")
        if name == joint_name:
            return i
    raise ValueError(f"Joint not found: {joint_name}")


def get_joint_limits_degrees(p, robot_id: int, joint_id: int) -> tuple[float, float]:
    joint_info = p.getJointInfo(robot_id, joint_id)
    lower_limit = joint_info[8]
    upper_limit = joint_info[9]
    return math.degrees(lower_limit), math.degrees(upper_limit)


def create_frame_items(p) -> list[int]:
    item_ids = []
    for _ in range(3):
        item_ids.append(p.addUserDebugLine([0, 0, 0], [0, 0, 0], [1, 1, 1], lineWidth=5))
        item_ids.append(p.addUserDebugText("", [0, 0, 0], [1, 1, 1], textSize=1.0))
    return item_ids


def draw_link_frame(
    p,
    robot_id: int,
    link_id: int,
    item_ids: list[int],
    length: float = 1,
) -> None:
    state = p.getLinkState(robot_id, link_id, computeForwardKinematics=True)
    origin = state[4]
    rotation = p.getMatrixFromQuaternion(state[5])

    x_axis = [rotation[0], rotation[3], rotation[6]]
    y_axis = [rotation[1], rotation[4], rotation[7]]
    z_axis = [rotation[2], rotation[5], rotation[8]]

    axes = [
        (x_axis, [1, 0, 0], "X"),
        (y_axis, [0, 1, 0], "Y"),
        (z_axis, [0, 0.25, 1], "Z"),
    ]

    for index, (axis, color, label) in enumerate(axes):
        end = [origin[i] + length * axis[i] for i in range(3)]
        p.addUserDebugLine(
            origin,
            end,
            color,
            lineWidth=6,
            replaceItemUniqueId=item_ids[index * 2],
        )
        p.addUserDebugText(
            label,
            end,
            color,
            textSize=2.0,
            replaceItemUniqueId=item_ids[index * 2 + 1],
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
        cameraDistance=2.8,
        cameraYaw=45,
        cameraPitch=-20,
        cameraTargetPosition=[0.0, 0.0, 0.7],
    )

    p.setGravity(0, 0, 0)
    p.loadURDF("plane.urdf")
    robot_id = p.loadURDF(
        str(urdf_path),
        basePosition=[0, 0, 0.85],
        baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
        useFixedBase=USE_FIXED_BASE,
        flags=p.URDF_USE_INERTIA_FROM_FILE,
    )

    print(f"Loaded {urdf_path.name} as body id {robot_id}.")

    right_elbow = get_joint_index(p, robot_id, "right_elbow_joint")
    right_shoulder = get_joint_index(p, robot_id, "right_shoulder_roll_joint")
    shoulder_frame = create_frame_items(p)
    elbow_frame = create_frame_items(p)
    shoulder_min, shoulder_max = get_joint_limits_degrees(p, robot_id, right_shoulder)
    elbow_min, elbow_max = get_joint_limits_degrees(p, robot_id, right_elbow)
    shoulder_slider = p.addUserDebugParameter(
        "right shoulder roll (degrees)",
        -120,
        0,
        0,
    )
    elbow_slider = p.addUserDebugParameter(
        "right elbow angle (degrees)",
        -60,
        90,
        90,
    )

    try:
        while p.isConnected():
            shoulder_angle = math.radians(p.readUserDebugParameter(shoulder_slider))
            elbow_angle = math.radians(p.readUserDebugParameter(elbow_slider))
            p.resetJointState(robot_id, right_elbow, elbow_angle)
            p.resetJointState(robot_id, right_shoulder, shoulder_angle)

            draw_link_frame(p, robot_id, right_shoulder, shoulder_frame)
            draw_link_frame(p, robot_id, right_elbow, elbow_frame)
            p.stepSimulation()
            time.sleep(1.0 / 240.0)
    except KeyboardInterrupt:
        pass
    finally:
        if p.isConnected():
            p.disconnect()


if __name__ == "__main__":
    main()