import serial
from tkinter import *
import binascii
import sys
import cv2
import time
import numpy as np
import multiprocessing
from multiprocessing import Pool
import keyboard

dic = {
    '0': 0, '1': 1, '2': 2,
    '3': 3, '4': 4, '5': 5,
    '6': 6, '7': 7, '8': 8,
    '9': 9, 'a': 10, 'b': 11,
    'c': 12, 'd': 13, 'e': 14,
    'f': 15,
}


def setChars(OData):
    # 将字符串转换为字符组格式，并还原每一行
    asd = []
    i = 0
    TLong = len(OData)
    while OData[i + 1] is not None:
        asd.append(OData[i:i + 2])
        i = i + 2
        if i >= TLong:
            break
    return asd


def byte16ToInt(byte16):
    # 补码转换为原码
    if (byte16 & 0x80) == 0:
        r = byte16
    else:
        byte16 = byte16 ^ 0xff
        byte16 = byte16 + 1
        r = -byte16
    return r


def NumChange2(OData):
    TLong = len(OData)
    i = 0
    asd = []
    while TRUE:
        a = 0x00 + 16 * dic[OData[i][0]] + dic[OData[i][1]]
        b = byte16ToInt(a)
        asd.append(b)
        i = i + 1
        if i == TLong:
            break
    return asd


def reverse(data, limit):
    if data > limit:
        data = data - 2*limit
        data = round(data, 3)
    return data

def searchFF1(OData):
    # rool pitch yaw
    x_angle, y_angle, z_angle = 0, 0, 0
    x_acc, y_acc, z_acc = 0, 0, 0
    x_gyro, y_gyro, z_gyro = 0, 0, 0
    # 检索角度数据并反馈数据流
    for i in range(len(OData) - 12):
        if OData[i:i + 3] == ["55", "55", "01"] and i + 11 <= len(OData):
            Slop = OData[i + 4:i + 10]
            for m in range(len(Slop)):
                Slop[m] = byte16ToInt(0x00 + 16 * dic[Slop[m][0]] + dic[Slop[m][1]])
            sloops = setSloop(Slop)
            x_angle, y_angle, z_angle = sloops[0], sloops[1], sloops[2]
            OData[:i] = []
        if OData[i:i + 3] == ["55", "55", "03"] and i + 17 <= len(OData):
            Slop = OData[i + 4:i + 16]
            for m in range(len(Slop)):
                Slop[m] = byte16ToInt(0x00 + 16 * dic[Slop[m][0]] + dic[Slop[m][1]])
            sloops = getAccGyro(Slop)
            x_acc, y_acc, z_acc, x_gyro, y_gyro, z_gyro = sloops[0], sloops[1], sloops[2], sloops[3], sloops[4], sloops[5]
            OData[:i] = []

    x_angle = reverse(x_angle, 180)
    y_angle = reverse(y_angle, 180)
    z_angle = reverse(z_angle, 180)
    x_acc = reverse(x_acc, 4)  # 量程4G
    y_acc = reverse(y_acc, 4)
    z_acc = reverse(z_acc, 4)
    x_gyro = reverse(x_gyro, 500)  # 量程500
    y_gyro = reverse(y_gyro, 500)
    z_gyro = reverse(z_gyro, 500)


    return x_angle, y_angle, z_angle, x_acc, y_acc, z_acc, x_gyro, y_gyro, z_gyro


def setSloop(OData):
    # 角度算法，数组有三个模块，分别为x,y,z方向的角度
    return [round((byte16ToInt(OData[1]) << 8 | byte16ToInt(OData[0])) / 32768 * 180, 3),
            round((byte16ToInt(OData[3]) << 8 | byte16ToInt(OData[2])) / 32768 * 180, 3),
            round((byte16ToInt(OData[5]) << 8 | byte16ToInt(OData[4])) / 32768 * 180, 3)]

def getAccGyro(OData):
    # 加速度与角速度   ACC 4 / Gyro 2000分别为用户设置量程
    return [round((byte16ToInt(OData[1]) << 8 | byte16ToInt(OData[0])) / 32768 * 4, 3),
            round((byte16ToInt(OData[3]) << 8 | byte16ToInt(OData[2])) / 32768 * 4, 3),
            round((byte16ToInt(OData[5]) << 8 | byte16ToInt(OData[4])) / 32768 * 4, 3),
            round((byte16ToInt(OData[7]) << 8 | byte16ToInt(OData[6])) / 32768 * 500, 3),
            round((byte16ToInt(OData[9]) << 8 | byte16ToInt(OData[8])) / 32768 * 500, 3),
            round((byte16ToInt(OData[11]) << 8 | byte16ToInt(OData[10])) / 32768 * 500, 3),
            ]


def getSloop(OData):
    # 得到角度度数
    if OData:
        sys.stdout.write(
            "X轴的角度为：" + str(OData[0]) + "\t" + "Y轴的角度为：" + str(OData[1]) + "\t" + "Z轴的角度为：" + str(OData[2]) + "\n")
        return True
    return False


def imu_process(com_id, port_rate, save_path):
    print("开始测试：")
    ser = serial.Serial(com_id, port_rate)
    print("获取句柄成功，进入循环：")
    count = 0
    data = []
    array_data = []
    cnt = 0
    while True:
        count += 1
        # if count == 100:
        #     break
        Fdata = str(binascii.b2a_hex(ser.readline()))
        Fdata = Fdata[2:-1]
        Fdata = setChars(Fdata)
        data.extend(Fdata)
        curtime = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        x_angle, y_angle, z_angle, x_acc, y_acc, z_acc, x_gyro, y_gyro, z_gyro = searchFF1(data)
        if (x_angle, y_angle, z_angle) != (0, 0, 0) and (x_acc, y_acc, z_acc) != (0, 0, 0) and (x_gyro, y_gyro, z_gyro) != (0, 0, 0):
            array_data.append([time.time(), curtime, x_angle, y_angle, z_angle, x_acc, y_acc, z_acc, x_gyro, y_gyro, z_gyro])
            print("x_angle：" + str(x_angle) + "°\t" + "y_angle：" + str(y_angle) + "°\t" + "z_angle：" + str(z_angle) + "°"
                  + " x_acc：" + str(x_acc) + "G\t" + "y_acc：" + str(y_acc) + "G\t" + "z_acc：" + str(z_acc) + "G"
                  + " x_gyro：" + str(x_gyro) + "°/s\t" + "y_gyro：" + str(y_gyro) + "°/s\t" + "z_gyro：" + str(z_gyro) + "°/s")
            # np.save(save_path + str(time.time()) + ".npy", np.array(array_data[-1]))
            cnt+=1
            if cnt > 100:
                cnt = 0
                np.save(save_path+str(time.time())+".npy", np.array(array_data))
                array_data = []
        if keyboard.is_pressed("q"):
            break
    # array_data = np.array(array_data)
    # np.save(save_path, array_data)

def cap_process(cap_id, save_path):
    # VideoCapture方法是cv2库提供的读取视频方法
    cap = cv2.VideoCapture(cap_id)
    # 设置需要保存视频的格式“xvid”
    # 该参数是MPEG-4编码类型，文件名后缀为.avi
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    # 设置视频帧频
    fps = cap.get(cv2.CAP_PROP_FPS)
    # 设置视频大小
    cap.set(3, 1920)  # width=1920
    cap.set(4, 1080)  # height=1080
    size = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    # VideoWriter方法是cv2库提供的保存视频方法
    # 按照设置的格式来out输出
    out = cv2.VideoWriter(save_path, fourcc, fps, size)

    # font
    font = cv2.FONT_HERSHEY_SIMPLEX
    # org
    org = (50, 100)
    # fontScale
    fontScale = 1
    # Blue color in BGR
    color = (255, 255, 255)
    # Line thickness of 2 px
    thickness = 2
    # Using cv2.putText() method

    # 确定视频打开并循环读取
    cnt = 0
    while (cap.isOpened()):
        # 逐帧读取，ret返回布尔值
        # 参数ret为True 或者False,代表有没有读取到图片
        # frame表示截取到一帧的图片
        ret, frame = cap.read()
        if ret == True:
            # 垂直翻转矩阵
            # frame = cv2.flip(frame,0)

            # 当前时间格式化
            curtime = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
            frame = cv2.putText(frame, curtime, org, font,
                                fontScale, color, thickness, cv2.LINE_AA)
            frame = cv2.putText(frame, "frame:"+str(cnt), (50, 50), font,
                                fontScale, color, thickness, cv2.LINE_AA)

            out.write(frame)
            cv2.imshow('frame', frame)
            cnt += 1
            cv2.waitKey(1)
            if keyboard.is_pressed("q"):
                break
        else:
            break

    # 释放资源
    cap.release()
    out.release()
    # 关闭窗口
    cv2.destroyAllWindows()

def gps_process(com_id, port_rate, save_path):
    print("开始测试：")
    ser = serial.Serial(com_id, port_rate)
    print("获取句柄成功，进入循环：")
    array_data = []
    cnt = 0
    while True:
        info = str(ser.readline())
        if "$GNGGA" in info.split(',')[0]:
            cnt += 1
            curtime = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
            array_data.append([time.time(), curtime])
            array_data[-1].extend(info.split(',')[1:12])
            print(array_data[-1])
            if cnt > 10:
                cnt = 0
                np.save(save_path+str(time.time())+".npy", np.array(array_data))
                array_data = []
        if keyboard.is_pressed("q"):
            break
    # array_data = np.array(array_data)


if __name__ == '__main__':
    # imu_process("com8", 9600, "data/record_new/steering_wheel.npy")
    pool = Pool(processes=4)
    pool.apply_async(func=imu_process, args=("com8", 9600, "data/record_new/steering_wheel/"))
    pool.apply_async(func=imu_process, args=("com3", 115200, "data/record_new/imu/"))
    pool.apply_async(func=cap_process, args=(1, 'data/record_new/video.avi'))
    pool.apply_async(func=gps_process, args=("com9", 38400, "data/record_new/gps/"))
    pool.close()
    pool.join()

