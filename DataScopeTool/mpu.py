


from pyqtgraph.Qt import QtGui, QtCore

import numpy as np

import pyqtgraph as pg

import serial
import threading
from time import  sleep

import re

# 实例化一个绘图窗口

win = pg.GraphicsWindow()
win.resize(1000, 600)
win.setWindowTitle('mpu6050 data')
win.setBackground('w')

# 启用抗锯齿选项
pg.setConfigOptions(antialias=True)

# 添加一个图形
p6 = win.addPlot(title="绘图数据更新")

curve1 = p6.plot(pen='r')  # 图形使用黄色画笔进行绘制
curve2 = p6.plot(pen='g')  # 互补滤波
curve3 = p6.plot(pen='b')  #  kalman


data1 = []
data2 = []
data3 = []




if __name__ == '__main__':

    import sys

    # 定义一个更新函数
    uart = serial.Serial("COM11", 230400 , timeout=0.008)


    def uart_recv_thread():
        while True:
            raw_data = uart.read_all()

            if raw_data:
                data: str = raw_data.decode()

                match_obj = re.search(r"X\s+\((.*)\)", data)
                if match_obj:
                    x = match_obj.group(1).strip().split("\t")
                    print(f'X:{x}')

                match_obj = re.search(r"Y\s+\((.*)\)", data)
                if match_obj:
                    y = match_obj.group(1).strip().split('\t')
                    print(f"Y:{y}")

                data1.append(float(x[0]))
                data2.append(float(x[2]))
                data3.append(float(x[3]))

            sleep(0.01)



    recv_thread = threading.Thread(target=uart_recv_thread, daemon=True)
    recv_thread.start()


    def update():
        while True:
            sleep(0.05)
            curve1.setData((data1))  # 设置图形的数据值
            curve2.setData((data2))  # 设置图形的数据值
            curve3.setData((data3))  # 设置图形的数据值

    update_thread = threading.Thread(target=update, daemon=True)
    update_thread.start()





    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):

        QtGui.QApplication.instance().exec_()