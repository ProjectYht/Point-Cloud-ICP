import open3d as o3d
from draw import Draw3D

def o3d_ICP(src,tar,input_trans):
    print('o3d_icp:\n')
    # np转换为o3d需要的格式
    source = o3d.geometry.PointCloud(points=o3d.utility.Vector3dVector(src))
    target = o3d.geometry.PointCloud(points=o3d.utility.Vector3dVector(tar))

    print('初始变换矩阵：\n',input_trans)
    Draw3D(source, target, input_trans,'匹配前：')

    # 迭代次数可以调高，这里为了显示清楚，迭代次数设置比较小
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source, target, 0.02, input_trans,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=140))

    output_trans = reg_p2p.transformation
    print("输出变换矩阵：\n", output_trans)
    Draw3D(source, target, output_trans,'匹配后：')