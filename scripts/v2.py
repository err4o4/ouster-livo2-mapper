#!/usr/bin/env python3
import numpy as np
import json

def to_mat44(flat_list):
    arr = np.array(flat_list, dtype=float)
    if arr.size != 16:
        raise ValueError("Expected 16 numbers for a 4x4 matrix.")
    return arr.reshape(4, 4)

def invert_se3(T):
    """Inverse of an SE(3) transform assuming last row [0 0 0 1]."""
    R = T[:3, :3]
    t = T[:3, 3]
    R_inv = R.T
    t_inv = -R_inv @ t
    T_inv = np.eye(4)
    T_inv[:3, :3] = R_inv
    T_inv[:3, 3] = t_inv
    return T_inv

def is_rotation_matrix(R, tol=1e-6):
    should_be_I = R.T @ R
    I = np.eye(3)
    return np.allclose(should_be_I, I, atol=tol) and np.isclose(np.linalg.det(R), 1.0, atol=1e-6) or np.isclose(np.linalg.det(R), -1.0, atol=1e-6)

def main():
    # === Paste your JSON here (or load from a file/CLI as desired) ===
    imu_to_sensor_json = {"imu_to_sensor_transform": [1, 0, 0, -2.441, 0, 1, 0, -9.725, 0, 0, 1, 7.533, 0, 0, 0, 1]}
    lidar_to_sensor_json = {"lidar_to_sensor_transform": [-1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 38.195, 0, 0, 0, 1]}

    T_S_I = to_mat44(imu_to_sensor_json["imu_to_sensor_transform"])   # Sensor <- IMU
    T_S_L = to_mat44(lidar_to_sensor_json["lidar_to_sensor_transform"]) # Sensor <- LiDAR

    # Compute IMU <- LiDAR
    T_I_S = invert_se3(T_S_I)
    T_I_L = T_I_S @ T_S_L

    extrinsic_R = T_I_L[:3, :3]
    extrinsic_T = T_I_L[:3, 3]

    # Sanity checks
    if not np.allclose(T_S_I[3], [0, 0, 0, 1], atol=1e-9) or not np.allclose(T_S_L[3], [0, 0, 0, 1], atol=1e-9):
        raise ValueError("Input matrices must be homogeneous with last row [0 0 0 1].")
    if not is_rotation_matrix(T_S_I[:3,:3]) or not is_rotation_matrix(T_S_L[:3,:3]):
        print("Warning: One of the input rotation parts isn't perfectly orthonormal. Proceed with caution.")
    if not is_rotation_matrix(extrinsic_R):
        print("Warning: Output rotation is not perfectly orthonormal (numerical issue or bad inputs).")

    np.set_printoptions(precision=6, suppress=True)
    print("T_imu_lidar (IMU <- LiDAR):\n", T_I_L)
    print("\nextrinsic_R (rotation of LiDAR wrt IMU):\n", extrinsic_R)
    print("\nextrinsic_T (translation of LiDAR wrt IMU):\n", extrinsic_T)

if __name__ == "__main__":
    main()