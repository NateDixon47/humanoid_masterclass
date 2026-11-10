import numpy as np

E = np.eye(3)
a_hat = np.array([
    [0, -1, 0],
    [1, 0, 0],
    [0, 0, 0]
])

omega = np.radians(90)
t = 1

e = E + a_hat * np.sin(omega * t) + a_hat @ a_hat * (1 - np.cos(omega * t))
p = np.array([0.3, 0.0, 0.0])
p_final = e @ p
print(p_final)