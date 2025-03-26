import time
import os
import numpy as np
import math
import sys

from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
from board import SCL, SDA
import busio

import rclpy                                # ROS2 Python接口库
from rclpy.node import Node                 # ROS2 节点类
from interfaces.msg import Output           # 字符串消息类型
from interfaces.srv import Command

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

Maxstep = 40000
NUM_SERVOS = 18
_count = 1000

class OSC:
    def __init__(self):
        self.Y = np.zeros(_count + Maxstep)

class CPG:
    def __init__(self):
        # 初始化CPG的數據
        self.osc = [OSC() for _ in range(5)]  # osc有5個元素

leg = [CPG() for _ in range(7)]  # 6個腿+1個（從1開始）

def walk(x, a, b):
    #a:調整右髖關節擺幅
    #b:調整左髖關節擺幅
    #c:調整左、右膝關節擺幅
    #d:調整左、右踝關節擺幅
    #e:調整R0、R2踝關節擺幅
    #f:調整L0、L2踝關節擺幅
    #a=1
    #b=1
    c=1
    d=1
    #e=1
    print(f"a={a}\tb={b}\n")
    print(f"c={c}\td={d}\n")
    #print(f"e={e}\n")
    servos["R00"].angle=(cpg_deg_change(leg[1].osc[1].Y[x] * a))
    servos["R01"].angle=(cpg_deg_change(leg[1].osc[2].Y[x] * c))
    servos["R02"].angle=(cpg_deg_change(leg[1].osc[3].Y[x] * d))
    servos["R10"].angle=(cpg_deg_change(leg[2].osc[1].Y[x] * a))
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
    print("load cpg")
    file_paths = [
        f"/home/user/ros2_obf_ws/src/cpg/fixed_cpg/YYout{i}{j}.txt" for i in range(1, 7) for j in range(1, 4)
    ]
    files = [open(path, "r") for path in file_paths]

    # 讀取數據
    for count in range(1, Maxstep + 1):
        for i in range(1, 7):  # 腿 1 至 6
            for j in range(1, 4):  # 關節 1 至 3
                file_index = (i - 1) * 3 + (j - 1)
                if files[file_index]:  # 確保文件開啟
                    value = float(files[file_index].readline().strip())
                    #print(f"count={count}")
                    leg[i].osc[j].Y[count] = (value)

    # 關閉文件
    for file in files:
        file.close()
    turn(Maxstep)

def cpg_deg_change(rad):
    deg = rad * (180 / math.pi)
    return deg+90

def turn(num_count):
    #控制膝關節永遠>=90度
    #控制leg[2,5]的osc[3]>=90度
    #leg[1,2,3]的osc[2]*-1
    #leg[4,5,6]的osc[1]*-1
    #leg[1,2,3]的osc[3]*-1
    #leg[1,6]的osc[3]*-1
    for i in range(num_count + 1):
        for j in range(1,7):
            if leg[2].osc[3].Y[i] < 0:
                leg[2].osc[3].Y[i] = 0
            if leg[5].osc[3].Y[i] < 0:
                leg[5].osc[3].Y[i] = 0
            if leg[j].osc[2].Y[i] < 0:
                leg[j].osc[2].Y[i] = 0
            if j<=3:
                leg[j].osc[2].Y[i]=-leg[j].osc[2].Y[i]
                leg[j].osc[3].Y[i]=-leg[j].osc[3].Y[i]
            else:
                leg[j].osc[1].Y[i]=-leg[j].osc[1].Y[i]
        leg[1].osc[3].Y[i]=-leg[1].osc[3].Y[i]
        leg[6].osc[3].Y[i]=-leg[6].osc[3].Y[i]

        """ 
        #leg[2,5]的osc[1]要*-1      
        leg[2].osc[1].Y[i]=-leg[2].osc[1].Y[i] 
        leg[5].osc[1].Y[i]=-leg[5].osc[1].Y[i] 
        """
 

    
    
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

class Servo(Node):

    def __init__(self, name):
        super().__init__(name)
        self.cli = self.create_client(Command, 'command')
        while not self.cli.wait_for_service(timeout_sec=0.05):
            self.get_logger().info('service not available, waiting again...')
        self.req = Command.Request()
        timer_period = 0.02
        self.timer = self.create_timer(timer_period, self.callback)
        self.last_execution_time = time.time()
        self.step = 0
        self.a = 1
        self.b = 1
        self.max_step = Maxstep  # 保存 Maxstep
    def callback(self):
        current_time = time.time()
        interval = current_time - self.last_execution_time
        self.get_logger().info(f'interval: {interval:.4f} (s)')
        self.last_execution_time = current_time
        self.step+=1
        print(f"step:{self.step}\n")
        if ((leg[1].osc[1].Y[self.step - 1] <= 0 and leg[1].osc[1].Y[self.step] > 0) or
            (leg[1].osc[1].Y[self.step - 1] >= 0 and leg[1].osc[1].Y[self.step] < 0)):
            print("fuzzy-----------------------------------------------------\n")
            future = self.send_request() #發送reduest
        walk(self.step,self.a,self.b)

    def send_request(self):
        print("send_request\n")
        self.future = self.cli.call_async(self.req)
        self.future.add_done_callback(self.handle_response)
    def handle_response(self, future):
        try:
            response = future.result()
            self.a=response.a
            self.b=response.b
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


def main():                               # ROS2节点主入口main函数
    load_cpg()
    Servo_initialization()

    rclpy.init()              # ROS2 Python接口初始化
    node = Servo("servo")  # 创建ROS2节点对象并进行初始化
    
    rclpy.spin(node)

    node.destroy_node()                            # 销毁节点对象
    rclpy.shutdown()                               # 关闭ROS2 Python接口

if __name__ == '__main__':
    main()
    

