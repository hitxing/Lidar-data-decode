"""
实现批量修改文件的名字，按照顺序排列，并且改成六位数的形式,文件后缀名可以自行指定，用zfill()方法补齐，达到6位
但是这个程序有个bug就是，输入和输出的目录只能是同一个目录，否则会出现找不到文件的错误
现在在这个基础上增加一个功能，将改好名字的文件剪切到新的文件目录里
"""
# --** coding="UTF-8" **--

import os
import re
import sys
import shutil

###需要重命名的bin文件的目录
input_path = r"D:\Coop_Precept\lidar_process\txt2bin\bin/"

###命名好后保存bin文件的目录
# output_path = r"D:\Coop_precept\Complex-YOLOv4-Pytorch-master\dataset\kitti\testing\velodyne\O-B-1618\未翻转的"
output_path = r"D:\Coop_Precept\lidar_process\txt2bin\bin/"

###
def rename(input_path):
    """
    下面这个函数的功能是实现批量修改名字，把原本按照帧数和时间戳命名的bin文件命名为6位补齐的名字，保存到原有的目录下
    """
    fileList = os.listdir(input_path)
    print("start rename...")

    # 得到进程当前工作目录
    currentpath = os.getcwd()

    if not os.path.exists(output_path):
        os.makedirs(output_path)

    # 将当前工作目录修改为待修改文件夹的位置
    os.chdir(input_path)

    # 名称变量起始值
    num = 0
    # 遍历文件夹中所有文件
    for fileName in fileList:
        # 文件重新命名
        os.rename(fileName, (str(num).zfill(6) + '.bin'))
        # 改变编号，继续下一项
        num = num + 1
    print("end...")
    # 改回程序运行前的工作目录
    os.chdir(currentpath)
    # 刷新
    sys.stdin.flush()

def remove_file(old_path, new_path):
    """
    
    该函数的功能是把改好名的bin文件，批量移动到新的目录下，
    旧的目录下会清空，如果不想清空，只想copy，把move改为copy即可
    """
    print(old_path)
    print(new_path)
    filelist = os.listdir(old_path) #列出该目录下的所有文件,listdir返回的文件列表是不包含路径的。
    print(filelist)
    for file in filelist:
        src = os.path.join(old_path, file)
        dst = os.path.join(new_path, file)
        # print('src:', src)
        # print('dst:', dst)
        shutil.move(src, dst)    ##不想清空旧目录，这里改为copy，否则默认清空就是move
    print("完成文件移动操作！！")
 
if __name__ == '__main__':
    ###以下代码是进行改名，把bin文件改为000000.bin的格式，并且按递增的顺序排列
    rename(input_path)

    ###以下代码是把命名好的bin文件移动或者复制到需要保存的目录下
    remove_file(input_path, output_path)
 






