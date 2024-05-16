# 简单实现ICP
from sklearn.neighbors import KDTree
from trans import Transform
import numpy as np
import time

def ICP(source, target, init_trans, max_iters=80, tolerance=1e-6):
    """
    参数：
    source: 源点云，每行一个点的坐标。
    target: 目标点云，每行一个点的坐标。
    max_iters: 最大迭代。
    tolerance: 收敛判定的容差。

    返回值：
    align_source: 对齐后的源点云。
    """
    print('初始变换矩阵：\n', init_trans)
    # 构建 KD 树
    tree = KDTree(target)

    # 利用初始参数得到匹配点云
    align_source = Transform(source, init_trans)

    oput_trans = init_trans  # 在初始参数基础上累积的齐次矩阵

    new_trans = np.eye(4)  # 每次迭代新增齐次矩阵

    prev_error = np.inf
    time_list,trans_list=[],[]
    for iteration in range(max_iters):
        T1=time.time()

        distances, indices = tree.query(align_source)
        new_target = target[indices].reshape((len(target), 3))

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
        new_trans[:3, :3] = new_R
        new_trans[:3, 3] = new_T
        T2=time.time()


        # 更新 匹配点云，误差，齐次矩阵
        align_source = np.dot(align_source, new_R.T) + new_T.T
        prev_error = error
        oput_trans = np.dot(new_trans, oput_trans)
        time_list.append(T2-T1)
        trans_list.append(oput_trans)

    with open('iteration_result.txt','w') as f:
        for i in range(len(time_list)):
            f.write('迭代次数：{}，用时：{}\n'.format(i,time_list[i]))
            f.write('旋转平移矩阵：\n{}\n\n'.format(trans_list[i]))


    
    print("输出变换矩阵：\n", oput_trans)
    return oput_trans
