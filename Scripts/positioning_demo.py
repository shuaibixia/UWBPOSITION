import numpy as np 
import json  # 读取基站位置文件
from Lib.trilateration import solve_position_3d  # 导入lib 中的三边定位算法

def load_anchors(UwbConfig_path="/home/rsp/UwbProject/UwbConfig/anchors.json"):#读取基站配置文件，返回 numpy 数组 (N,3) 方便后面快速运算
    with open(UwbConfig_path, "r") as f:
        data = json.load(f)
    anchorslist= []
    for a in data["anchors"]: #遍历anchors 里面的所有坐标信息 转为 np 数组方便运算
        anchorslist.append([a["x"], a["y"], a["z"]])
    anchors = np.array(anchorslist, dtype=float)
    return anchors

def simulate_ranges(anchors, true_pos, noise_std=100.0):
    """模拟UWB 测距 并且加入高斯噪声10cm的偏移 这是为了应对实际上的无人机室内漂移的问题"""
    dists = np.linalg.norm(anchors - true_pos, axis=1)
    noise = np.random.normal(0, noise_std, size=dists.shape) #噪声生成的参数 0 均值 10cm 标准差 形状和 dist 一样
    return dists + noise

def main():
    #读取基站配置
    anchors = load_anchors()
    print("Loaded anchors:\n", anchors)

    #定义无人机真实位置（单位 mm） """后期这里直接输入 UWB 基站的实际参数 接受存在 DATA 的数据以及在 lib 中的处理函数传进来就好 """
    true_pos = np.array([10000.0, 1000.0, 1000.0])  # (x,y,z)
    print("True position:", true_pos)

    #模拟测距
    ranges = simulate_ranges(anchors, true_pos)
    print("Simulated ranges (mm):", ranges)

    #调用三边定位算法
    est_pos = solve_position_3d(anchors, ranges)
    print("Estimated position:", est_pos)

    #计算误差
    error = np.linalg.norm(est_pos - true_pos)
    print(f"定位误差: {error:.2f} mm")

if __name__ == "__main__":
    main()