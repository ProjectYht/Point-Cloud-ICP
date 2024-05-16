import matplotlib.pyplot as plt
import open3d as o3d
import copy


def Draw3D(source, target, transform, winname):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transform)
    o3d.visualization.draw_geometries(
        [source_temp, target_temp], window_name=winname)
