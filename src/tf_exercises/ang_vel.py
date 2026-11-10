import numpy as np

omega = np.array([0, 0, 2])
P = np.array([0.0, 0.4, 0.0])

print(np.cross(omega, P))