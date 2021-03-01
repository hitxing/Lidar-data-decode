# --** coding="UTF-8" **--

import os
import re
import sys
import shutil

###需要重命名的bin(或pcd)文件的目录  
input_path = r".\txt2bin\bin/"

###命名好后保存bin(或pcd)文件的目录
output_path = r".\txt2bin\bin/"

###
def rename(input_path):
    """
    下面这个函数的功能是实现批量修改名字，把原本按照帧数和时间戳命名的bin文件(或者pcd文件)命名为6位补齐的名字，保存到原有的目录下
    """
    fileList = os.listdir(input_path)
    print("start rename...")

    currentpath = os.getcwd()

    if not os.path.exists(output_path):
        os.makedirs(output_path)

    os.chdir(input_path)

    num = 0
    for fileName in fileList:
        os.rename(fileName, (str(num).zfill(6) + '.bin'))   ## if you want to rename your pcd files, replace '.bin' to '.pcd'
        num = num + 1
    print("end...")
    os.chdir(currentpath)
    sys.stdin.flush()

def remove_file(old_path, new_path):
    """
    
    该函数的功能是把改好名的bin文件，批量移动到新的目录下，
    旧的目录下会清空，如果不想清空，只想copy，把move改为copy即可
    """
    print(old_path)
    print(new_path)
    filelist = os.listdir(old_path) 
    # print(filelist)
    for file in filelist:
        src = os.path.join(old_path, file)
        dst = os.path.join(new_path, file)
        shutil.move(src, dst)    ##不想清空旧目录，这里改为copy，否则默认清空就是move
 
if __name__ == '__main__':
    rename(input_path)
    remove_file(input_path, output_path)
 






