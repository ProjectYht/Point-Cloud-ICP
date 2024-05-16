import open3d as o3d
from plyfile import PlyData
import numpy as np
from draw import Draw3D



def down_sample(pcd):
    # 设置降采样到的点的数量
    num = 10000
    pcd_down = pcd.farthest_point_down_sample(num)
    return pcd_down

# 使用重心化 标准差进行初始平移和缩放
def ply2pcd(ply):
    vertex = ply['vertex']
    x, y, z = vertex['x'], vertex['y'], vertex['z']
    P = np.column_stack((x, y, z))
    pcd = o3d.geometry.PointCloud(points=o3d.utility.Vector3dVector(P))
    return pcd

def test(pcd1,pcd2):
    pcd_down1=down_sample(pcd1)
    pcd_down2=down_sample(pcd2)

    points1 = np.asarray(pcd_down1.points)
    points2 = np.asarray(pcd_down2.points)

    # 计算每个坐标维度（x、y、z）的标准差
    std_dev1 = np.std(points1, axis=0)
    std_dev2 = np.std(points2, axis=0)


    print("Standard deviation in x:", std_dev2[0]/std_dev1[0])
    print("Standard deviation in y:", std_dev2[1]/std_dev1[1])
    print("Standard deviation in z:", std_dev2[2]/std_dev1[2])

def GetKeyPoint(pcd):
    
    # 使用ISS特征检测器检测特征点
    keypoint_detector = o3d.geometry.ISSKeypoint()
    keypoints = keypoint_detector.compute(pcd)

    return keypoints


def Draw(pcd, keyP):
    pcd.paint_uniform_color([1, 0.706, 0])
    keyP.paint_uniform_color([0, 0.1, 1])

    o3d.visualization.draw_geometries([keyP, pcd])


ply1 = PlyData.read('DPEX_Data12/hand-low-tri.ply')
ply2 = PlyData.read('DPEX_Data12/hand-high-tri.ply')
pcd1 = ply2pcd(ply1)
pcd2 = ply2pcd(ply2)


test(pcd1,pcd2)
