import numpy as np
import json

# Input JSON strings (replace with your actual sensor output if needed)
imu_to_sensor_json = '{"imu_to_sensor_transform": [1, 0, 0, -2.441, 0, 1, 0, -9.725, 0, 0, 1, 7.533, 0, 0, 0, 1]}'
lidar_to_sensor_json = '{"lidar_to_sensor_transform": [-1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 38.195, 0, 0, 0, 1]}'

imu_to_sensor = np.array(json.loads(imu_to_sensor_json)["imu_to_sensor_transform"]).reshape(4, 4)
lidar_to_sensor = np.array(json.loads(lidar_to_sensor_json)["lidar_to_sensor_transform"]).reshape(4, 4)

# Compute LiDAR->IMU transform: T_LI = T_LS * inv(T_IS)
T_LI = lidar_to_sensor @ np.linalg.inv(imu_to_sensor)

# Extract rotation and translation
R_LI = T_LI[:3, :3]
T_LI_t = T_LI[:3, 3] / 1000.0  # convert mm → m

# Print results
print("extrinsic_R:")
print(np.round(R_LI, 6))
print("\nextrinsic_T (m):")
print(np.round(T_LI_t, 6))

# Output in FAST-LIVO YAML format
R_flat = [float(f"{v:.6f}") for v in R_LI.flatten()]
T_flat = [float(f"{v:.6f}") for v in T_LI_t]

print("\nYAML block for FAST-LIVO2:")
print("extrinsic_R:", R_flat)
print("extrinsic_T:", T_flat)
