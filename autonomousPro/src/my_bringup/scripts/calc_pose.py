import numpy as np
import math

def estimate_target_position(rect, depth, robot_pose):
    # 默认相机内参
    fx = 600.0
    fy = 600.0
    cx = 320.0
    cy = 240.0

    # 1. 计算 rect 中心像素
    (x1, y1), (x2, y2) = rect
    u = (x1 + x2) / 2
    v = (y1 + y2) / 2

    # 2. 像素坐标 + 深度 -> 相机坐标系 (Z前, X右, Y下)
    z = depth
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy

    # 相机坐标 [右, 下, 前] -> base_link 假设相机在前方 0.1m，高度 0.2m，朝前
    pt_cam = np.array([x, y, z, 1.0])
    T_cam_to_base = np.array([
        [1, 0, 0,  0.1],   # 向前 0.1m
        [0, 1, 0,  0.0],   # 不偏移左右
        [0, 0, 1,  0.2],   # 高度 0.2m
        [0, 0, 0,  1.0]
    ])
    pt_base = T_cam_to_base @ pt_cam

    # 3. base_link -> map/world，使用 robot_pose 旋转和平移
    x_r, y_r = robot_pose['x'], robot_pose['y']
    yaw = robot_pose['yaw']

    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)

    x_map = x_r + pt_base[0] * cos_yaw - pt_base[1] * sin_yaw
    y_map = y_r + pt_base[0] * sin_yaw + pt_base[1] * cos_yaw

    return x_map, y_map
