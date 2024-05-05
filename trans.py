
# 旋转矩阵和欧拉角转换
import numpy as np

# 测试旋转矩阵和欧拉角转换
# Euler = [9.58, 5.35, 5.44]
# print('输入欧拉角:\n', Euler, '\n')
# R_mat = EulerToR_mat(Euler)
# print('旋转矩阵：\n', R_mat, '\n')
# Euler = R_matToEuler(R_mat)
# print('欧拉角：\n', Euler, '\n')


def EulerToR_mat(angles):
    # 角度转换为弧度
    radians = np.radians(angles)

    Rx = np.array([[1, 0, 0],
                   [0, np.cos(radians[0]), -np.sin(radians[0])],
                   [0, np.sin(radians[0]), np.cos(radians[0])]])

    Ry = np.array([[np.cos(radians[1]), 0, np.sin(radians[1])],
                   [0, 1, 0],
                   [-np.sin(radians[1]), 0, np.cos(radians[1])]])

    Rz = np.array([[np.cos(radians[2]), -np.sin(radians[2]), 0],
                   [np.sin(radians[2]), np.cos(radians[2]), 0],
                   [0, 0, 1]])
    R = np.dot(Rz, np.dot(Ry, Rx))
    return R


def isR_mat(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


def R_matToEuler(R):
    assert (isR_mat(R))
    sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6
    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0
    # 弧度转换为角度
    angles = np.degrees([x, y, z])
    return angles


# 旋转平移变换
def RT(points, angles, translations):

    R = EulerToR_mat(angles)
    R_P = np.dot(R, points.T)
    RT_P = R_P + np.array(translations).reshape(3, 1)
    return RT_P.T

# 使用齐次旋转平移矩阵变换


def Transform(points, transform):
    homo_points = np.hstack((points, np.ones((len(points), 1))))
    temp_points = np.dot(transform, homo_points.T)
    new_points = temp_points[:3, :].T

    return new_points
