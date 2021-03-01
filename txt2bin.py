"""
功能：把pcap包解析出来的txt文件，批量转换为bin格式文件并保存
function： convert txt files to bin format
"""

import os
import sys
import os.path
import pickle
import struct

## 在这里设置你的txt文件目录，如下所示   set the floder of your txt files here as follows
dirroot = r".\txt2bin\txt/"

## 在这里设置你的bin文件目录，如下所示   set the floder of your bin files here as follows
newdirroot=r".\txt2bin\bin/"

for dirnames in os.listdir(dirroot):
    print ("进入文件夹:"+ dirroot)
    #print dirnames
    if dirnames.split('.')[-1]!='txt':
        continue

    bin_filename=dirnames.split('.txt')[0] +'.bin'
   
    txt_file=open(dirroot + dirnames,'r') 
    bin_file=open(newdirroot + bin_filename,'wb')

    lines=txt_file.readlines()
    for j,line in enumerate(lines):
        if j == 0:
            continue
        curLine=line.split(',')[1:4]                    # txt文件中，[1:4]代表x,y,z    in txt files, [1:4] respent for x,y,z
        curLine.append(line.split(',')[5])              # txt文件中，[5]代表intensity  in txt files, [5] respent for intensity
        for i in range(len(curLine)):
            if len(curLine[i])==0:
                continue
            if i == 3:                                                # intensity
                parsedata = struct.pack("f",float(curLine[i]))        ## 这里可选择intensity用浮点数形式还是整型格式  you can choose intensity in float or int format
                # parsedata = struct.pack("i",int(curLine[i]))

                bin_file.write(parsedata)
            else:                                                     
                parsedata = struct.pack("f",float(curLine[i]))         
                bin_file.write(parsedata)

    bin_file.close()
    txt_file.close()
