# Fill in each origin xyz from rpo_model.urdf -- replace every None below.
hip_yaw_origin   = (-0.071093, 0.0725, -0.051778)   # left_hip_yaw_joint
hip_roll_origin  = (-0.017693, 0.0, -0.072346)   # left_hip_roll_joint
hip_pitch_origin = (0.060622, 0.02085, -0.035)   # left_hip_pitch_joint
knee_origin      = (0.0, 0.0, -0.25)   # left_knee_joint

chain = (hip_yaw_origin, hip_roll_origin, hip_pitch_origin, knee_origin)
predicted = tuple(sum(v[i] for v in chain) for i in range(3))

print(f"Computed base_link -> left_knee_link translation: "
      f"({predicted[0]:.5f}, {predicted[1]:.5f}, {predicted[2]:.5f})")