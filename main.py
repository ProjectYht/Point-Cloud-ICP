from trans import EulerToR_mat, R_matToEuler, RT
from o3d_match import o3d_ICP
from match import ICP
from plyfile import PlyData
import numpy as np

# 打开.ply文件
plydata = PlyData.read('DPEX_Data12/hand-low-tri.ply')

# 获取点云数据
vertex = plydata['vertex']
x, y, z = vertex['x'], vertex['y'], vertex['z']

# 点云P
P = np.column_stack((x, y, z))
# 点云P->待匹配目标点云
RT_P = RT(P, [9.5, 5.3, 0], [3.4, 2.7, 0])

# 设置初始旋转·平移参数
init_euler = [9.4, 5.3, 0]
init_translation = [3.3, 2.6, 0]
init_R_mat = EulerToR_mat(init_euler)
init_T_mat = np.array([init_translation]).T
init_trans = np.vstack(
    (np.hstack((init_R_mat, init_T_mat)), np.array([[0, 0, 0, 1]])))


print('open3d ICP working:')
o3d_transform=o3d_ICP(P, RT_P, init_trans)


print('my ICP working:')
my_transform = ICP(P, RT_P, init_trans)

print('精度评定：')
print('旋转 平移参数真值：',init_euler,init_translation)
print('o3d ICP 输出参数：',R_matToEuler(o3d_transform[:3,:3]), o3d_transform[:3,3])
print('my ICP 输出参数：',R_matToEuler(my_transform[:3,:3]), my_transform[:3,3])

