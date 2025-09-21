# UwbProject/Lib/trilateration.py. #三边定位算法块
import numpy as np  #导入NumPy库题
from scipy.optimize import least_squares  #导入scipy库，least_squares用来做非线性最小二乘法

def residuals_3d(p, anchors, ranges):  #残差 P：当前无人机位置 anchors(N,3)：基站位置 ranges(N,)：UWB 测出来的位置
    return np.linalg.norm(anchors - p, axis=1) - ranges  # 计算每个基站到P 点的距离(调用距离公式)减去测距，返回误差向量

def solve_position_3d(anchors, ranges, initial=None, enforce_positive_z=True):
    """三维最小二乘定位函数
    anchors: (N,3) numpy array 每个基站坐标 单位米
    ranges:  (N,) numpy array 对应测距 单位米
    initial: None 或 (3,) 初始猜测位置
    enforce_positive_z: 若 True 且解 z < 0 强制取正
    返回: est (3,) [x, y, z] 估计位置"""
    anchors = np.asarray(anchors, dtype=float)  # 确保基站坐标为 float 数组
    ranges = np.asarray(ranges, dtype=float)    # 确保测距为 float 数组
    if initial is None:
        initial = np.array([10000.0, 1000.0, 1000.0])  # 给一个安全的初始猜测 # 初始猜测为000 即为一号基站基站，z 偏高 1 米
        initial[2] = min(max(initial[2], 0), 6000)  # 强制 z 在 [0,6000] 内 否则下面的 bound 中会因为相加后超过 不在范围内报错
    res = least_squares(residuals_3d,initial,args=(anchors, ranges),method='trf',
    bounds=([None, None, 0.0],   #下限，z ≥ 0
            [None, None, 6000]),  #上限，z ≤ 6
    max_nfev=1000
    )  # 用LM求最小残差 max防止死循环
    est = res.x  # 获取优化结果
    if enforce_positive_z and est[2] < 0:  # 如果 z 小于 0 且要求强制为正
        est[2] = abs(est[2])  # 取绝对值，防止高度为负
    return est  # 返回最终三维位置

# #
# #这是个测试注释
# #这里是使用气压计或者是IMU测得的高度来定位计算 先求二维平面 再求结合高度完成定位 备选方案（未修改单位 还是米 要改成毫米）

# def solve_position_with_known_z(anchors, ranges, z_known): 
#     anchors = np.asarray(anchors, dtype=float)  # 转为 float 数组
#     ranges = np.asarray(ranges, dtype=float)    # 转为 float 数组
#     z_i = anchors[:,2]  # 获取所有基站的 z 坐标
#     radicands = ranges**2 - (z_known - z_i)**2  # 计算水平投影半径平方
#     if np.any(radicands <= 0):  # 检查可行性，如果某个值 <=0;  
#         raise ValueError("某些基站在该高度下不可达d^2 - dz^2 <= 0 检查 z 或 ranges")  # 抛出异常
#     r_proj = np.sqrt(radicands)  # 水平投影半径
#     xy_anchors = anchors[:,:2]  # 取基站 XY 坐标;  
#     def residuals_2d(p, xy_anchors, r_proj):
#         return np.linalg.norm(xy_anchors - p, axis=1) - r_proj  # 计算水平平面误差向量
#     initial = xy_anchors.mean(axis=0)  # 初始猜测为 XY 平均位置
#     res = least_squares(residuals_2d, initial, args=(xy_anchors, r_proj), method='lm', max_nfev=2000)  # 最小二乘优化 XY
#     xy = res.x  # 获取优化结果 XY
#     return np.array([xy[0], xy[1], z_known])  # 返回最终位置，Z 使用已知高度