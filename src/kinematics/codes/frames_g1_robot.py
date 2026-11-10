from __future__ import annotations

import math
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parent
DESCRIPTION_DIR = ROOT / "g1_description"
DEFAULT_URDF = DESCRIPTION_DIR / "g1_29dof.urdf"


def get_joint_index(p, robot_id: int, joint_name: str) -> int:
    for i in range(p.getNumJoints(robot_id)):
        name = p.getJointInfo(robot_id, i)[1].decode("utf-8")
        if name == joint_name:
            return i
    raise ValueError(f"Joint not found: {joint_name}")


def create_vector_items(p) -> dict[str, int]:
    return {
        "pa_line": p.addUserDebugLine([0, 0, 0], [0, 0, 0], [1, 0, 0], lineWidth=6),
        "r_line": p.addUserDebugLine([0, 0, 0], [0, 0, 0], [1, 0, 0], lineWidth=6),
        "ph_line": p.addUserDebugLine([0, 0, 0], [0, 0, 0], [1, 0, 0], lineWidth=6),
        "pa_text": p.addUserDebugText("", [0, 0, 0], [1, 0, 0], textSize=1.6),
        "r_text": p.addUserDebugText("", [0, 0, 0], [1, 0, 0], textSize=1.6),
        "ph_text": p.addUserDebugText("", [0, 0, 0], [1, 0, 0], textSize=1.6),
    }


def create_axis_items(p) -> list[int]:
    item_ids = []
    for _ in range(3):
        item_ids.append(p.addUserDebugLine([0, 0, 0], [0, 0, 0], [1, 1, 1], lineWidth=4))
        item_ids.append(p.addUserDebugText("", [0, 0, 0], [1, 1, 1], textSize=1.4))
    return item_ids


def create_panel_item(p) -> int:
    return p.addUserDebugText("", [0, 0, 0], [0, 0, 0], textSize=1.0)


def format_vector(name: str, vector: list[float]) -> str:
    return f"{name} = [{vector[0]: .3f}, {vector[1]: .3f}, {vector[2]: .3f}]"


def draw_world_axes(p, item_ids: list[int], length: float = 0.35) -> None:
    axes = [
        ([length, 0, 0], [1, 0, 0], "x"),
        ([0, length, 0], [0, 0.6, 0], "y"),
        ([0, 0, length], [0, 0.25, 1], "z"),
    ]
    for index, (end, color, label) in enumerate(axes):
        p.addUserDebugLine(
            [0, 0, 0],
            end,
            color,
            lineWidth=4,
            replaceItemUniqueId=item_ids[index * 2],
        )
        p.addUserDebugText(
            label,
            end,
            color,
            textSize=1.4,
            replaceItemUniqueId=item_ids[index * 2 + 1],
        )


def draw_vectors(
    p,
    item_ids: dict[str, int],
    shoulder_position: tuple[float, float, float],
    hand_position: tuple[float, float, float],
) -> None:
    world_origin = [0.0, 0.0, 0.0]
    pa = list(shoulder_position)
    ph = list(hand_position)
    r_mid = [(pa[i] + ph[i]) * 0.5 for i in range(3)]
    pa_mid = [0.5 * pa[i] for i in range(3)]
    ph_mid = [0.5 * ph[i] for i in range(3)]

    p.addUserDebugLine(
        world_origin,
        pa,
        [0.85, 0.0, 0.0],
        lineWidth=7,
        replaceItemUniqueId=item_ids["pa_line"],
    )
    p.addUserDebugLine(
        pa,
        ph,
        [1.0, 0.15, 0.0],
        lineWidth=7,
        replaceItemUniqueId=item_ids["r_line"],
    )
    p.addUserDebugLine(
        world_origin,
        ph,
        [0.55, 0.0, 0.0],
        lineWidth=7,
        replaceItemUniqueId=item_ids["ph_line"],
    )

    p.addUserDebugText(
        "p_a",
        pa_mid,
        [0.85, 0.0, 0.0],
        textSize=1.7,
        replaceItemUniqueId=item_ids["pa_text"],
    )
    p.addUserDebugText(
        "r",
        r_mid,
        [1.0, 0.15, 0.0],
        textSize=1.7,
        replaceItemUniqueId=item_ids["r_text"],
    )
    p.addUserDebugText(
        "p_h",
        ph_mid,
        [0.55, 0.0, 0.0],
        textSize=1.7,
        replaceItemUniqueId=item_ids["ph_text"],
    )


def update_component_panel(
    p,
    panel_id: int,
    shoulder_position: tuple[float, float, float],
    hand_position: tuple[float, float, float],
) -> None:
    pa = list(shoulder_position)
    ph = list(hand_position)
    r = [(pa[i] + ph[i]) for i in range(3)]
    check = [pa[i] + r[i] for i in range(3)]
    panel_text = "\n".join(
        [
            "Vector components [m]",
            format_vector("p_a", pa),
            format_vector("r  ", r),
            format_vector("p_h", ph),
            format_vector("p_a + r", check),
        ]
    )
    p.addUserDebugText(
        panel_text,
        [0.95, -0.45, 1.25],
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
        cameraDistance=2.5,
        cameraYaw=35,
        cameraPitch=-18,
        cameraTargetPosition=[0.0, -0.05, 0.75],
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

    shoulder_slider = p.addUserDebugParameter(
        "right shoulder roll phi (degrees)",
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

    vector_items = create_vector_items(p)
    axis_items = create_axis_items(p)
    panel_id = create_panel_item(p)

    print("Move the shoulder slider and observe how p_h changes direction.")

    try:
        while p.isConnected():
            phi = math.radians(p.readUserDebugParameter(shoulder_slider))
            elbow_angle = math.radians(p.readUserDebugParameter(elbow_slider))
            p.resetJointState(robot_id, shoulder_joint, phi)
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

            draw_world_axes(p, axis_items)
            draw_vectors(p, vector_items, shoulder_state[4], hand_state[4])
            update_component_panel(p, panel_id, shoulder_state[4], hand_state[4])
            time.sleep(1.0 / 240.0)
    except KeyboardInterrupt:
        pass
    finally:
        if p.isConnected():
            p.disconnect()


if __name__ == "__main__":
    main()
