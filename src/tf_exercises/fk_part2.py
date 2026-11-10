import math

def rot_y(theta, v):
    """Rotate a 3-vector by theta radians about the Y axis."""
    x, y, z = v
    c, s = math.cos(theta), math.sin(theta)
    return (x * c + z * s, y, -x * s + z * c)

knee_origin = (0.0, 0.0, -0.25)          # left_thigh_pitch_link -> left_knee_link (fixed)
ankle_pitch_origin = (0.0, -0.02085, -0.3)  # left_knee_link -> left_ankle_pitch_link (rotates with left_knee_joint)

for angle_deg in (0, 45, 90):
    theta = math.radians(angle_deg)
    rotated_ankle = rot_y(theta, ankle_pitch_origin)
    predicted = tuple(knee_origin[i] + rotated_ankle[i] for i in range(3))
    print(f"left_knee_joint = {angle_deg:>3} deg  ->  left_thigh_pitch_link -> left_ankle_pitch_link ~= "
          f"({predicted[0]:.3f}, {predicted[1]:.3f}, {predicted[2]:.3f})")