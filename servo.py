import time
import os
import numpy as np
import math

from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
from board import SCL, SDA
import busio

import rclpy                                # ROS2 Python接口库
from rclpy.node import Node                 # ROS2 节点类
from interfaces.msg import Output           # 字符串消息类型

# 設定I2C通信
i2c = busio.I2C(SCL, SDA)

# 初始化右邊和左邊的PCA9685板
pcaR = PCA9685(i2c, address=0x41)
pcaL = PCA9685(i2c, address=0x40)

# 設置頻率
pcaR.frequency = 50
pcaL.frequency = 50

# 將所有馬達存儲在字典中
servos = {}  # 用字典儲存伺服馬達

SERVOS = [
    "R00", "R01", "R02", "R10", "R11", "R12", "R20", "R21", "R22",
    "L00", "L01", "L02", "L10", "L11", "L12", "L20", "L21", "L22"
]

_Maxstep = 40200
NUM_SERVOS = 18
_count = 1000

class OSC:
    def __init__(self):
        self.Y = np.zeros(_count + _Maxstep)

class CPG:
    def __init__(self):
        # 初始化CPG的數據
        self.osc = [OSC() for _ in range(5)]  # osc有5個元素

leg = [CPG() for _ in range(7)]  # 6個腿+1個（從1開始）

def walk(x, a, b):
    c=1
    d=0

    servos["R00"].angle=(cpg_deg_change(leg[1].osc[1].Y[x] * a))
    servos["R01"].angle=(cpg_deg_change(leg[1].osc[2].Y[x] * c))
    servos["R02"].angle=(cpg_deg_change(leg[1].osc[3].Y[x] * d))
    servos["R10"].angle=(cpg_deg_change(leg[2].osc[1].Y[x] * a)-10)
    servos["R11"].angle=(cpg_deg_change(leg[2].osc[2].Y[x] * c))
    servos["R12"].angle=(cpg_deg_change(leg[2].osc[3].Y[x] * d))
    servos["R20"].angle=(cpg_deg_change(leg[3].osc[1].Y[x] * a))
    servos["R21"].angle=(cpg_deg_change(leg[3].osc[2].Y[x] * c))
    servos["R22"].angle=(cpg_deg_change(leg[3].osc[3].Y[x] * d))
    servos["L00"].angle=(cpg_deg_change(leg[6].osc[1].Y[x] * b))
    servos["L01"].angle=(cpg_deg_change(leg[6].osc[2].Y[x] * c))
    servos["L02"].angle=(cpg_deg_change(leg[6].osc[3].Y[x] * d))
    servos["L10"].angle=(cpg_deg_change(leg[5].osc[1].Y[x] * b))
    servos["L11"].angle=(cpg_deg_change(leg[5].osc[2].Y[x] * c))
    servos["L12"].angle=(cpg_deg_change(leg[5].osc[3].Y[x] * d))
    servos["L20"].angle=(cpg_deg_change(leg[4].osc[1].Y[x] * b))
    servos["L21"].angle=(cpg_deg_change(leg[4].osc[2].Y[x] * c))
    servos["L22"].angle=(cpg_deg_change(leg[4].osc[3].Y[x] * d))

def load_cpg():
    file_paths = [
        f"/home/user/ros2_obf_ws/src/servo/servo/fixed_cpg/YYout{i}{j}.txt" for i in range(1, 7) for j in range(1, 5)
    ]
    files = [open(path, "r") for path in file_paths]

    # 讀取數據
    for count in range(1, _Maxstep + 1):
        for i in range(1, 7):  # 腿 1 至 6
            for j in range(1, 4):  # 關節 1 至 3
                file_index = (i - 1) * 4 + (j - 1)
                if files[file_index]:  # 確保文件開啟
                    value = float(files[file_index].readline().strip())
                    leg[i].osc[j].Y[count] = (value)

    # 關閉文件
    for file in files:
        file.close()
    turn(_Maxstep)

def cpg_deg_change(rad):
    deg = rad * (180 / math.pi)
    return deg+90

def turn(num_count):
    pass
    # 控制膝關節永遠>=90
    for i in range(num_count + 1):
        for j in range(1, 7):
            if leg[j].osc[2].Y[i] < 0:
                leg[j].osc[2].Y[i] = 0
    #leg[1,2,3]的osc[2]*-1
    #leg[4,5,6]的osc[1]*-1
    #leg[2,5]的osc[1]要*-1
    for i in range(num_count + 1):
        for j in range(1,7):
            if j<=3:
                leg[j].osc[2].Y[i]=-leg[j].osc[2].Y[i]
                leg[j].osc[3].Y[i]=-leg[j].osc[3].Y[i]
            else:
                leg[j].osc[1].Y[i]=-leg[j].osc[1].Y[i]
"""         leg[2].osc[1].Y[i]=-leg[2].osc[1].Y[i] 
        leg[5].osc[1].Y[i]=-leg[5].osc[1].Y[i] """
 

    
    
def Servo_initialization():
    for i, motor_name in enumerate(SERVOS[:9]):  # 右邊的9個馬達
        servo_motor = servo.Servo(pcaR.channels[i], min_pulse=600, max_pulse=2400)
        servo_motor.angle = 90  # 初始角度設為90度
        servos[motor_name] = servo_motor

    # 初始化左邊的伺服馬達
    for i, motor_name in enumerate(SERVOS[9:]):  # 左邊的9個馬達
        servo_motor = servo.Servo(pcaL.channels[i], min_pulse=600, max_pulse=2400)
        servo_motor.angle = 90  # 初始角度設為90度
        servos[motor_name] = servo_motor

class SubscriberNode(Node):

    def __init__(self, name):
        super().__init__(name)                             # ROS2节点父类初始化
        self.sub = self.create_subscription(Output, "output", self.listener_callback, 10) # 创建订阅者对象（消息类型、话题名、订阅者回调函数、队列长度）

    def listener_callback(self, msg):                      # 创建回调函数，执行收到话题消息后对数据的处理
        self.get_logger().info('I heard: step=%d,a=%lf,b=%lf' % (msg.step,msg.a,msg.b)) # 输出日志信息，提示订阅收到的话题消息
        walk(msg.step, msg.a, msg.b) 

def main(args=None):                               # ROS2节点主入口main函数
    load_cpg()
    Servo_initialization()
    rclpy.init(args=args)                          # ROS2 Python接口初始化
    node = SubscriberNode("output")  # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                               # 循环等待ROS2退出
    node.destroy_node()                            # 销毁节点对象
    rclpy.shutdown()                               # 关闭ROS2 Python接口
    
    

