import numpy as np

# Shoulder rotation angle
theta = np.radians(30)

# Transformation matrix (shoulder frame to world frame)
# T = np.array([
#     [1, 0,              0,             0.2],
#     [0, np.cos(theta), -np.sin(theta), 0.1],
#     [0, np.sin(theta),  np.cos(theta), 1.0],
#     [0, 0,              0,             1]
# ])

# # Hand position in shoulder frame (homogeneous coordinates)
# p_shoulder = np.array([0, 0, -0.4, 1])

phi = np.radians(30)
psi = np.radians(40)

Tw_s = np.array([
    [1, 0,              0,             0.2],
    [0, np.cos(theta), -np.sin(theta), 0.1],
    [0, np.sin(theta),  np.cos(theta), 1.0],
    [0, 0,              0,             1]
])
# Ts_e = np.array([
#     [1, 0,              0,             0.0],
#     [0, np.cos(phi), -np.sin(phi), -0.3],
#     [0, np.sin(phi),  np.cos(phi), 0.0],
#     [0, 0,              0,             1]
# ])

Ts_e = np.array([
    [np.cos(psi), -np.sin(psi),  0, 0.0],
    [np.sin(psi), np.cos(psi),   0, -0.3],
    [0,           0,             0, 0.0],
    [0,           0,             0, 1]
])

p_eh = np.array([0, -0.25, 0, 1])

# Transform hand position into world frame
p_world = Tw_s @ Ts_e @ p_eh

print("Hand position in world frame:")
print(p_world[:3])