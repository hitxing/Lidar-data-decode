import mayavi.mlab
import torch
import numpy as np
import glob 
import math, os
import cv2


### 在这里更改你的点云存放目录  set the folder of your bin files
point_path = r'.\visualization/'

def lidar_lists(lidar_path):

    files = sorted(glob.glob("%s/*.bin" % lidar_path))
    id_lists = [os.path.split(x)[1].split(".")[0].strip() for x in files]
    num_samples = id_lists.__len__()
    print("读取样本个数为：%d 个" % num_samples)
    return id_lists

def get_lidar(lidar_path, idx):
   
    lidar_file = os.path.join(lidar_path, '%06d.bin' % int(idx))
    assert os.path.exists(lidar_file)
    points = np.fromfile(lidar_file, dtype = np.float32).reshape(-1, 4)
    points = torch.from_numpy(points)
    print("********************************")
    print("当前帧点云的shape为：", points.shape)
    return points

def viz_mayavi(points,vals="distance"):
    x=points[:,0]
    y=points[:,1]
    z=points[:,2]
    
    d=torch.sqrt(x**2+y**2)

    if vals=="height": 
        col=z
    else:
        col=d

    fig=mayavi.mlab.figure(bgcolor=(0,0,0),size=(1280,720))
    mayavi.mlab.points3d(x,y,z,
                         col,
                         mode="point",
                         colormap='spectral',
                         figure=fig,
                         )

    mayavi.mlab.show()


if __name__=="__main__":

    # #批量可视化点云帧的代码
    idxs = lidar_lists(point_path)
    for idx in idxs:
        print(idx)
        test_point = get_lidar(point_path, int(idx))
        viz_mayavi(test_point, vals="height")

        ## 防止数据太多，只显示其中3帧，自行修改参数即可
        if int(idx) == 2:
            break

