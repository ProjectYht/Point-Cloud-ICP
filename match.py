# 简单实现ICP
from sklearn.neighbors import KDTree
from trans import RT
import numpy as np


def ICP(source, target, ini_angles, ini_trans, max_iters=20, tolerance=1e-4):
    """
    参数：
    source: 源点云，每行一个点的坐标。
    target: 目标点云，每行一个点的坐标。
    max_iters: 最大迭代。
    tolerance: 收敛判定的容差。

    返回值：
    align_source: 对齐后的源点云。
    """
    # 构建 KD 树
    tree = KDTree(target)

    # 利用初始参数得到匹配点云
    align_source = RT(source, ini_angles, ini_trans)

    prev_error = np.inf
    for iteration in range(max_iters):

        distances, indices = tree.query(align_source)
        new_target=target[indices].reshape((len(target),3))

        # 计算源点云和目标点云之间的误差
        error = np.mean(distances)
        # print(f"Iteration {iteration}: Error = {error}")

        # 跳出迭代
        if np.abs(prev_error - error) < tolerance or iteration == max_iters - 1:
            break

        # 重心化
        source_center = np.mean(align_source, axis=0)
        target_center = np.mean(new_target, axis=0)
        centered_source = align_source - source_center
        centered_target = new_target - target_center

        # 计算最佳刚体变换
        covariance = np.dot(centered_source.T, centered_target)
        u, _, vt = np.linalg.svd(covariance)
        new_R = np.dot(vt.T, u.T)
        new_T = target_center.T - np.dot(new_R, source_center.T)

        # 更新对齐点云，误差
        align_source = np.dot(align_source, new_R.T) + new_T.T
        prev_error = error

    return new_R, new_T
