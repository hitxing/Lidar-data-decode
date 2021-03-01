import os
import re
import datetime
import argparse
import dpkt
import math
import numpy as np

from gps import GprmcMessage, utc_to_weekseconds


### 这里设置的是雷达的水平修正角度值，具体的值可以从雷达的配置包中读取出来
A1 = 2.88
A2 = 8.43
A3 = 4.51
A4 = 6.87

def read_uint32(data, idx):
    ustime = data[idx] + data[idx+1]*256 + data[idx+2]*256*256 + data[idx+3]*256*256*256
    return ustime


class Lidar():
    def __init__(self):
        self.point_cloud = []

    def process_data_frame(self, data, frame_idx):
        raise NotImplementedError("Subclasses should implement this!")

    def process_position_frame(self, data, frame_idx):
        raise NotImplementedError("Subclasses should implement this!")


class VelodyneVLP16(Lidar):
    # factor distance centimeter value to meter
    FACTOR_CM2M = 0.01

    # factor distance value to cm, each velodyne distance unit is 2 mm  镭神是2.5mm
    # FACTOR_MM2CM = 0.2
    FACTOR_MM2CM = 0.25 

    def __init__(self, dual_mode=False):
        super(VelodyneVLP16, self).__init__()
        self.dual_mode = dual_mode
        self.timing_offsets = self.calc_timing_offsets()

        ##下面是旧款C的通道垂直角度
        # self.omega = np.array([-18, -15, -12, -10, -8, -7, -6, -5, -4, -3.33, -2.66, -3, -2.33, -2, -1.33, -1.66, -1, -0.66, 0, -0.33, 0.33, 0.66, 1.33, 1, 1.66, 2, 3, 4, 6, 8, 11, 14])
        
        ### 下面是旧款A型雷达的通道垂直角度
        self.omega = np.array([-16, -15, -14, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15])
        
        self.count_lasers = 32

    def calc_timing_offsets(self):
        timing_offsets = [[0.0 for x in range(12)] for y in range(32)]  # Init matrix  12*32  12个数据块，32个通道（两个16线叠加）

        #只计算单回波的值，双回波暂时不用管
        full_firing_cycle = 49.152  # μs  32个通道发光时间
        single_firing = 1.536  # μs     镭神的32线是1.536us，而且没有空闲时间

        for x in range(12):
            for y in range(32):
                if self.dual_mode:
                    dataBlockIndex = (x - (x % 2)) + int((y / 32))
                else:
                    dataBlockIndex = (x) + int((y / 32))
                dataPointIndex = y % 32
                timing_offsets[y][x] = \
                    (full_firing_cycle * dataBlockIndex) + (single_firing * dataPointIndex)
        ofset = np.array(timing_offsets).T/1000000
        return ofset  
       

    def process_data_frame(self, data, timestamp, frame_idx):
        """
        :param data: A velodyne packet consisting of 12 (n) blocks and 24 (m) sequences and 16 firing pre sequence
        :param frame_idx:
        :return: X,Y,Z-coordinate, azimuth, intensitiy, timestamp of each firing, sequence ordered, shape of each = [384x1]
        """
        data = np.frombuffer(data, dtype=np.uint8).astype(np.uint32)

        """ 
        #以下代码用于测试读取设备包中的水平偏置角度A1 得到结果为1.9°，与shark软件中解析出来的结果一致

        A1 = data[34:36]
        A1 = A1[1:]+A1[0:1]*256
        print("水平修正角度A1为：", A1*0.01)    
        """
        # veldyne has 12 blocks each 100 bytes data  这一点和雷神一样  每个block是100 bytes数据
        # data-legth = 1206 bytes
        blocks = data[0:1200].reshape(12, 100)       #这个是二维数组的迭代，迭代的index是每个数据块

        distances = []
        intensities = []
        azimuth_per_block = []
        # iteratie through each block   每个block只返回一个方位角值angle，共12个值(用来判断是否一个block扫描)，每个block外加32个距离值和32个反射强度值
        for i, blk in enumerate(blocks):
            dists, intens, angles = self.read_firing_data(blk)
            distances.append(dists)
            intensities.append(intens)
            azimuth_per_block.append(angles)


        azimuth_per_block = np.array(azimuth_per_block)

        ## Note: all these arrray have th same size, number of firing in one packet
        ## 12*32 算出每一个channel的水平角  把azimuth的维度转成和distances以及internsities的维度一致
        azimuth = self.calc_precise_azimuth(azimuth_per_block).reshape(12, 32)
        distances = np.array(distances)
        intensities = np.array(intensities)

        # now calculate the cartesian coordinate of each point
        X, Y, Z = self.calc_cart_coord(distances, azimuth)

        # calculating timestamp [microsec] of each firing   这里会进行numpy的广播，让timestamps的维度和xyz及distances一致
        timestamps = timestamp + self.timing_offsets
      
        X = X.flatten()
        Y = Y.flatten()
        Z = Z.flatten()
        intensities = intensities.flatten()
        azimuth = azimuth.flatten()
        timestamps = timestamps.flatten()
        distances = distances.flatten() * self.FACTOR_MM2CM * self.FACTOR_CM2M

        # remeber the last azimuth for roll over checking
        self.cur_azimuth = azimuth

        return X, Y, Z, intensities,azimuth, timestamps, distances

    def process_position_frame(self, data, frame_idx):
        """
        获取GPS位置的函数，用于处理GPS位置信息
        """
        timestamp = data[198:202]
        timestamp = read_uint32(timestamp, 0)

        ppps_status = data[202]

        gprmc = data[206:]
        gprmc = gprmc.split()[0]  # filter out gprmc message, remaining are zeros
        gprmc = gprmc.decode('ascii').split(',')  # convert bytes array to string

        print(gprmc)

        gps_msg = GprmcMessage()
        time = gprmc[1]
        date = gprmc[9]
        gps_msg.datetime = datetime.datetime.strptime(date + time, '%d%m%y%H%M%S')

        gps_msg.status = gprmc[2]
        gps_msg.lat = float(gprmc[3])
        gps_msg.lat_ori = gprmc[4]
        gps_msg.long = float(gprmc[5])
        gps_msg.long_ori = gprmc[6]
        gps_msg.velocity = float(gprmc[7])
        gps_msg.course = float(gprmc[8])

        gps_msg.mag = float(gprmc[10])
        gps_msg.mag_sign = gprmc[11]
        gps_msg.singularity = gprmc[12]
        gps_msg.weeks, _, gps_msg.seconds, _ = utc_to_weekseconds(gps_msg.datetime, leapseconds=0)

        return gps_msg


    def calc_precise_azimuth(self, azimuth):
        """
        Linear interpolation of azimuth values
        :param azimuth:
        :return:
        """
        org_azi = azimuth.copy()
        
        precision_azimuths = []
        # iterate through each block
        for n in range(12): # n=0..11
            azimuth = org_azi.copy()
            try:
                # First, adjust for an Azimuth rollover from 359.99° to 0°
                # 如果比上一个块的水平角要小，那就是转完一圈了
                if azimuth[n + 1] < azimuth[n]:
                    azimuth[n + 1] += 360.

                # Determine the azimuth Gap between data blocks
                azimuth_gap = azimuth[n + 1] - azimuth[n]
            except:
                if azimuth[n] < azimuth[n-1]:
                    azimuth[n] += 360
                azimuth_gap = azimuth[n] - azimuth[n-1]

            factor = azimuth_gap / 32.
           
            ### 各个通道的发光时间不是逐顺序递增的 这个是针对A型均匀雷达和旧款C型雷达而言的
            k = np.array([ 0,  2,  4,  6,  8,  10,  12,  14,  16,  18, 20, 22, 24, 26, 28, 30, 1, 3, 5, 7, 9, 11, 13, 15,
 17, 19, 21, 23, 25, 27, 29, 31])

            ### 下面是新款C型雷达的发光时间，按照通道id依次递增
#             k = np.array([ 0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23,
#  24, 25, 26, 27, 28, 29, 30, 31])

            precise_azimuth = azimuth[n] + factor * k

            ##加上水平修正角度  下面是A型1°均匀雷达的水平修正角度代码  水平修正角度每台雷达的都不一样，需要从对应的设备包中读取出来 一台1.9 一台1.8
            for i in range(32):
                if i%2==0:
                    precise_azimuth[i] = precise_azimuth[i] + 1.9
                else:
                    precise_azimuth[i] = precise_azimuth[i] - 1.9


            ###下面是新款C型雷达的水平修正角度代码
            # for i in range(32):
            #     if i == 29:
            #         precise_azimuth[i] = precise_azimuth[i] - A2
            #     elif i in (0, 1, 4, 8, 9, 12, 16, 17, 21, 24, 25):
            #         precise_azimuth[i] = precise_azimuth[i] + A2
            #     elif i in (5, 13, 20, 28):
            #         precise_azimuth[i] = precise_azimuth[i] + A4
            #     elif i in (7, 15, 22, 30):
            #         precise_azimuth[i] = precise_azimuth[i] + A3
            #     else:
            #         precise_azimuth[i] = precise_azimuth[i] + A1

            ###下面是旧款C型雷达的水平修正角度代码
            # for i in range(32):
            #     if i in (0, 2, 4, 6, 8, 12, 16, 20, 24, 26, 28, 30):
            #         precise_azimuth[i] = precise_azimuth[i] + A2
            #     elif i in (1, 3, 5, 7, 9, 13, 17, 21, 25, 27, 29, 31):
            #         precise_azimuth[i] = precise_azimuth[i] + A1
            #     elif i in (11, 15, 19, 23):
            #         precise_azimuth[i] = precise_azimuth[i] + A3
            #     else:
            #         precise_azimuth[i] = precise_azimuth[i] + A4

            precision_azimuths.append(precise_azimuth)

        precision_azimuths = np.array(precision_azimuths)
        # print("shape和类型：",precision_azimuths.shape, type(precision_azimuths))
        return precision_azimuths

    def calc_precise_azimuth_2(self, azimuth):
        org_azi = azimuth.copy()

        precision_azimuth = []
        # iterate through each block
        for n in range(12): # n=0..11
            azimuth = org_azi.copy()
            try:
                # First, adjust for an Azimuth rollover from 359.99° to 0°
                if azimuth[n + 1] < azimuth[n]:
                    azimuth[n + 1] += 360.

                # Determine the azimuth Gap between data blocks
                azimuth_gap = azimuth[n + 1] - azimuth[n]
            except:
                azimuth_gap = azimuth[n] - azimuth[n-1]

            # iterate through each firing
            for k in range(32):
                # Determine if you’re in the first or second firing sequence of the data block
                if k < 16:
                    # Interpolate
                    precise_azimuth = azimuth[n] + (azimuth_gap * 2.304 * k) / 55.296
                else:
                    # interpolate
                    precise_azimuth = azimuth[n] + (azimuth_gap * 2.304 * ((k-16) + 55.296)) / 55.296
                if precise_azimuth > 361.:
                    print("Error")
                print(precise_azimuth)
                precision_azimuth.append(precise_azimuth)
        precision_azimuth = np.array(precision_azimuth)
        return precision_azimuth

    def read_firing_data(self, data):
        block_id = data[0] + data[1]*256
        # 0xeeff is upper block
        assert block_id == 0xeeff

        azimuth = (data[2] + data[3] * 256) / 100   #这个返回的结果只是一个数据块首地址的方位角，需要做插值计算才能得到其余通道的方位角

        firings = data[4:].reshape(32, 3)
        distances = firings[:, 0] + firings[:, 1] * 256
        intensities = firings[:, 2]
        return distances, intensities, azimuth

    def calc_cart_coord(self, distances, azimuth):
        # convert distances to meters
        distances = distances * self.FACTOR_MM2CM * self.FACTOR_CM2M
        longitudes = np.tile(self.omega * np.pi / 180., 1)
        latitudes = azimuth * np.pi / 180.

        hypotenuses = distances * np.cos(longitudes)

        X = hypotenuses * np.sin(latitudes)
        Y = hypotenuses * np.cos(latitudes)
        Z = distances * np.sin(longitudes)
        return X, Y, Z
