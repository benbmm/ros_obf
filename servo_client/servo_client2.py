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
from interfaces.srv import CommandAdaption

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

#是否開啟適應地形
adaption_mode=1
#change是指在兩種模式切換:適應地形的時候踝關節固定，適應完則開啟踝關節，在controller node 中模糊控制器參數也會隨之切換
#當change_mode=1 代表change值會隨著adaption node回傳值變化。=0時則始終不變，因此不會切換控制器，踝關節也維持固定角度
change_mode=0
#當adaption node偵測到姿態變化，回傳chang=1，servo node傳給controller node
change=1

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
    #e:調整踝關節擺幅，在適應地形時需要將機身抬高
    
    c=1
    if (not change):
        d=1
    else:
        d=0
    
    if (adaption_mode==1):
        e=30
    else:
        e=0
    print(f"a={a}\tb={b}\n")
    #距離牆太近原地旋轉
    if (a==-2 and b==2):
        d=0
        a=-0.5
        b=0.5
    elif (a==2 and b==-2):
        d=0
        a=0.5
        b=-0.5
    print(f"c={c}\td={d}\n")
    a=1
    b=1
    #print(f"e={e}\n")
    servos["R00"].angle=(cpg_deg_change(leg[1].osc[1].Y[x] * a))
    servos["R01"].angle=(cpg_deg_change(leg[1].osc[2].Y[x] * c)+e)
    servos["R02"].angle=(cpg_deg_change(leg[1].osc[3].Y[x] * d)-e)
    servos["R10"].angle=(cpg_deg_change(leg[2].osc[1].Y[x] * a))
    servos["R11"].angle=(cpg_deg_change(leg[2].osc[2].Y[x] * c)+e)
    servos["R12"].angle=(cpg_deg_change(leg[2].osc[3].Y[x] * d)-e)
    servos["R20"].angle=(cpg_deg_change(leg[3].osc[1].Y[x] * a))
    servos["R21"].angle=(cpg_deg_change(leg[3].osc[2].Y[x] * c)+e)
    servos["R22"].angle=(cpg_deg_change(leg[3].osc[3].Y[x] * d)-e)
    servos["L00"].angle=(cpg_deg_change(leg[6].osc[1].Y[x] * b))
    servos["L01"].angle=(cpg_deg_change(leg[6].osc[2].Y[x] * c)-e)
    servos["L02"].angle=(cpg_deg_change(leg[6].osc[3].Y[x] * d)+e)
    servos["L10"].angle=(cpg_deg_change(leg[5].osc[1].Y[x] * b))
    servos["L11"].angle=(cpg_deg_change(leg[5].osc[2].Y[x] * c)-e)
    servos["L12"].angle=(cpg_deg_change(leg[5].osc[3].Y[x] * d)+e)
    servos["L20"].angle=(cpg_deg_change(leg[4].osc[1].Y[x] * b))
    servos["L21"].angle=(cpg_deg_change(leg[4].osc[2].Y[x] * c)-e)
    servos["L22"].angle=(cpg_deg_change(leg[4].osc[3].Y[x] * d)+e)

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
    #leg[2,5]的osc[3]交換
    #控制膝關節永遠>=90度
    #控制leg[2,5]的osc[3]>=90度
    #leg[1,2,3]的osc[2]*-1
    #leg[4,5,6]的osc[1]*-1
    #leg[1,2,3]的osc[3]*-1
    #leg[1,6]的osc[3]*-1
    
    for i in range(num_count + 1):

        temp=leg[2].osc[3].Y[i]
        leg[2].osc[3].Y[i]=leg[5].osc[3].Y[i]
        leg[5].osc[3].Y[i]=temp
        """ if leg[2].osc[3].Y[i] < 0:
            leg[2].osc[3].Y[i] = 0
        if leg[5].osc[3].Y[i] < 0:
            leg[5].osc[3].Y[i] = 0
        if leg[1].osc[3].Y[i] > 0:
            leg[1].osc[3].Y[i] = 0
        if leg[6].osc[3].Y[i] > 0:
            leg[6].osc[3].Y[i] = 0 """
        for j in range(1,7):
            #########
            if j==1 or j==6:
                if leg[j].osc[3].Y[i] > 0:
                    leg[j].osc[3].Y[i] = 0
            else:
                if leg[j].osc[3].Y[i] < 0:
                    leg[j].osc[3].Y[i] = 0
            #########
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
        if (i==4):
            #因為在pca9685上換腳位
            servo_motor = servo.Servo(pcaR.channels[9], min_pulse=600, max_pulse=2400)
        else:
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
            self.get_logger().info('controller_service not available, waiting again...')
        if(adaption_mode):
            self.cli_adaption = self.create_client(CommandAdaption, 'commandadaption')
            while not self.cli_adaption.wait_for_service(timeout_sec=0.05):
                self.get_logger().info('adaption_service not available, waiting again...')
        self.req = Command.Request()
        self.req_adaption = CommandAdaption.Request()
        timer_period = 0.02
        self.timer = self.create_timer(timer_period, self.callback)
        self.last_execution_time = time.time()
        self.step = 0
        #存左右擺幅控制量
        self.a = 1
        self.b = 1
        #存適應地形膝關節角度
        self.h = [0] * 8 
        self.max_step = Maxstep  # 保存 Maxstep
    def callback(self):
        self.step+=1
        #print(f"step:{self.step}\n")
        if(adaption_mode):
            self.send_request_adaption()
        if ((leg[1].osc[1].Y[self.step - 1] <= 0 and leg[1].osc[1].Y[self.step] > 0) or
            (leg[1].osc[1].Y[self.step - 1] >= 0 and leg[1].osc[1].Y[self.step] < 0)):
            #print("fuzzy-----------------------------------------------------\n")
            future = self.send_request() #發送reduest
        

        current_time = time.time()
        interval = current_time - self.last_execution_time
        #self.get_logger().info(f'interval: {interval:.4f} (s)')
        self.last_execution_time = current_time
        #self.get_logger().info(f'walk:step={self.step}')
        walk(self.step,self.a,self.b)

    def send_request(self):
        #print("send_request\n")
        self.req.get=change
        self.future = self.cli.call_async(self.req)
        self.future.add_done_callback(self.handle_response)
    def handle_response(self, future):
        try:
            response = future.result()
            self.a=response.a
            self.b=response.b
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
    
    #傳入adaption node的request，傳入step
    def send_request_adaption(self):
        #print("send_request_adaption\n")
        #self.get_logger().info(f'send_request_adaption:step={self.step+1}')
        self.req_adaption.step=self.step+1
        self.future_adaption = self.cli_adaption.call_async(self.req_adaption)
        self.future_adaption.add_done_callback(self.handle_response_adaption)

    #處理adaption node的response，回傳值h1~h6為膝關節的控制訊號，change值為是否有姿態變化
    def handle_response_adaption(self, future_adaption):
        try:
            response_adaptio = future_adaption.result()
            self.h[1]=response_adaptio.h1
            self.h[2]=response_adaptio.h2
            self.h[3]=response_adaptio.h3
            self.h[4]=response_adaptio.h4
            self.h[5]=response_adaptio.h5
            self.h[6]=response_adaptio.h6
            if (change_mode):
                global change
                change = response_adaptio.change
            #self.get_logger().info(f'h1: {response_adaptio.h1}\th2: {response_adaptio.h2}\th3: {response_adaptio.h3}\th4: {response_adaptio.h4}\th5: {response_adaptio.h5}\th6: {response_adaptio.h6}\nchange:{change}\tstep:{self.step}\n')
            if (self.step>0):
                for i in range(1,7):
                    #self.get_logger().info(f"i={i}")
                    #self.get_logger().info(f"leg[{i}].osc[2].Y[{self.step+1}]={leg[i].osc[2].Y[self.step+1]}")
                    #print(f"diff={self.h[i]-leg[1].osc[2].Y[self.step]}")
                    if (self.h[i] != leg[i].osc[2].Y[self.step+1]):
                        #self.get_logger().info(f"change,leg[{i}].osc[2].Y[{self.step+1}]={self.h[i]}")
                        leg[i].osc[2].Y[self.step+1]=self.h[i]
                    """ else:
                        self.get_logger().info("no change")
            else:
                self.get_logger().info("not yet change") """
        
                

        except Exception as e:
            self.get_logger().error(f'Servic_adaption call failed: {e}')


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
    

