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
from std_msgs.msg import Int64              # 發布 /timestep 用
from interfaces.msg import Output           # 字符串消息类型
from interfaces.msg import KneeAction       # 訂閱 /knee_action 用
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

adaptation_start_step=115

start_walking_step=100
# 是否開啟適應地形（保留原語義：控制 walk() 裡的 e=30 抬高機身）
adaption_mode = 1
# change 固定為 1，不再動態切換（原本的 adaption service 已改為 knee_controller 節點）
#change是指在兩種模式切換:適應地形的時候踝關節固定，適應完則開啟踝關節，在controller node 中模糊控制器參數也會隨之切換
#當change_mode=1 代表change值會隨著adaption node回傳值變化。=0時則始終不變，因此不會切換控制器，踝關節也維持固定角度
change_mode=0
#當adaption node偵測到姿態變化，回傳chang=1，servo node傳給controller node
change = 1


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
        d=-1
    
    if (adaption_mode==1):
        e=30
    else:
        e=0
    
    #距離牆太近原地旋轉
    if (a==-2 and b==2):
        d=0
        a=-0.5
        b=0.5
    elif (a==2 and b==-2):
        d=0
        a=0.5
        b=-0.5
    
    if(x<start_walking_step):
        servos["R00"].angle=(cpg_deg_change(0 * a))
        servos["R01"].angle=(cpg_deg_change(0 * c)+e)
        servos["R02"].angle=(cpg_deg_change(0 * d)-e-9)
        servos["R10"].angle=(cpg_deg_change(0 * a))
        servos["R11"].angle=(cpg_deg_change(0 * c)+e)
        servos["R12"].angle=(cpg_deg_change(0 * d)-e)
        servos["R20"].angle=(cpg_deg_change(0 * a))
        servos["R21"].angle=(cpg_deg_change(0 * c)+e)
        servos["R22"].angle=(cpg_deg_change(0 * d)-e-1)
        servos["L00"].angle=(cpg_deg_change(0 * b))
        servos["L01"].angle=(cpg_deg_change(0 * c)-e)
        servos["L02"].angle=(cpg_deg_change(0 * d)+e-7)
        servos["L10"].angle=(cpg_deg_change(0 * b))
        servos["L11"].angle=(cpg_deg_change(0 * c)-e)
        servos["L12"].angle=(cpg_deg_change(0 * d)+e-7)
        servos["L20"].angle=(cpg_deg_change(0 * b))
        servos["L21"].angle=(cpg_deg_change(0 * c)-e)
        servos["L22"].angle=(cpg_deg_change(0 * d)+e+1)
    else:
        servos["R00"].angle=(cpg_deg_change(leg[1].osc[1].Y[x] * a))
        servos["R01"].angle=(cpg_deg_change(leg[1].osc[2].Y[x] * c)+e)
        servos["R02"].angle=(cpg_deg_change(leg[1].osc[2].Y[x] * d)-e-9)
        servos["R10"].angle=(cpg_deg_change(leg[2].osc[1].Y[x] * a))
        servos["R11"].angle=(cpg_deg_change(leg[2].osc[2].Y[x] * c)+e)
        servos["R12"].angle=(cpg_deg_change(leg[2].osc[2].Y[x] * d)-e)
        servos["R20"].angle=(cpg_deg_change(leg[3].osc[1].Y[x] * a))
        servos["R21"].angle=(cpg_deg_change(leg[3].osc[2].Y[x] * c)+e)
        servos["R22"].angle=(cpg_deg_change(leg[3].osc[2].Y[x] * d)-e-1)
        servos["L00"].angle=(cpg_deg_change(leg[6].osc[1].Y[x] * b))
        servos["L01"].angle=(cpg_deg_change(leg[6].osc[2].Y[x] * c)-e)
        servos["L02"].angle=(cpg_deg_change(leg[6].osc[2].Y[x] * d)+e-7)
        servos["L10"].angle=(cpg_deg_change(leg[5].osc[1].Y[x] * b))
        servos["L11"].angle=(cpg_deg_change(leg[5].osc[2].Y[x] * c)-e)
        servos["L12"].angle=(cpg_deg_change(leg[5].osc[2].Y[x] * d)+e-7)
        servos["L20"].angle=(cpg_deg_change(leg[4].osc[1].Y[x] * b))
        servos["L21"].angle=(cpg_deg_change(leg[4].osc[2].Y[x] * c)-e)
        servos["L22"].angle=(cpg_deg_change(leg[4].osc[2].Y[x] * d)+e+1)

def load_cpg():
    print("load cpg")
    file_paths = [
        f"/home/user/ros2_obf_ws/src/cpg/knee_high_1.5/YYout{i}{j}.txt" for i in range(1, 7) for j in range(1, 4)
    ]
    files = [open(path, "r") for path in file_paths]

    # 讀取數據
    for count in range(1, Maxstep + 1):
        for i in range(1, 7):  # 腿 1 至 6
            for j in range(1, 4):  # 關節 1 至 3
                file_index = (i - 1) * 3 + (j - 1)
                if files[file_index]:  # 確保文件開啟
                    value = float(files[file_index].readline().strip())
                    leg[i].osc[j].Y[count] = (value)

    # 關閉文件
    for file in files:
        file.close()
    turn(Maxstep)

def cpg_deg_change(rad):
    deg = rad * (180 / math.pi)
    return deg+90

def turn(num_count):

    #前後左右方位相關
    #leg[1,6]的osc[3]*-1
    #leg[2,5]的osc[3]*-1

    #改變姿勢相關
    #leg[1~6]的osc[2]>=90度
    #leg[2,5]的osc[3]>=90度

    #馬達轉動方向相關
    #leg[1,2,3]osc[2]*-1
    #leg[1,2,3]的osc[3]*-1
    #leg[4,5,6]的osc[1]*-1

    for i in range(num_count + 1):
        #前後左右方位相關

        leg[1].osc[3].Y[i]=-leg[1].osc[3].Y[i]
        leg[6].osc[3].Y[i]=-leg[6].osc[3].Y[i]

        leg[2].osc[3].Y[i]=-leg[2].osc[3].Y[i]
        leg[5].osc[3].Y[i]=-leg[5].osc[3].Y[i] 

        #改變姿勢相關
        if leg[2].osc[3].Y[i] < 0:
            leg[2].osc[3].Y[i] = 0
        if leg[5].osc[3].Y[i] < 0:
            leg[5].osc[3].Y[i] = 0

        for j in range(1,7):
            if leg[j].osc[2].Y[i] < 0:
                leg[j].osc[2].Y[i] = 0
            if j<=3:
                #馬達轉動方向相關
                leg[j].osc[2].Y[i]=-leg[j].osc[2].Y[i]
                leg[j].osc[3].Y[i]=-leg[j].osc[3].Y[i]
            else:
                leg[j].osc[1].Y[i]=-leg[j].osc[1].Y[i]


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

        # ---- Service client：與 controller 節點通訊（取 a, b 擺幅）----
        self.cli = self.create_client(Command, 'command')
        while not self.cli.wait_for_service(timeout_sec=0.05):
            self.get_logger().info('controller_service not available, waiting again...')
        self.req = Command.Request()

        # ---- Publisher：發布 /timestep 給 knee_controller ----
        self.timestep_publisher = self.create_publisher(Int64, 'timestep', 10)

        # ---- Subscriber：訂閱 /knee_action（knee_controller 的輸出） ----
        self.knee_action_sub = self.create_subscription(
            KneeAction,
            'knee_action',
            self.knee_action_callback,
            10
        )

        # ---- 定時器（0.02s, 50Hz） ----
        timer_period = 0.02
        self.timer = self.create_timer(timer_period, self.callback)
        self.last_execution_time = time.time()

        # ---- 狀態變數 ----
        self.step = 0
        # 存左右擺幅控制量
        self.a = 1
        self.b = 1
        # 最新收到的 RL 膝關節修正量（6 條腿），初始為 0，收到第一筆前不影響 CPG
        # 順序：[L0, L1, L2, R0, R1, R2]
        self.latest_corrections = [0.0] * 6
        self.max_step = Maxstep  # 保存 Maxstep
        self.count_controller_server = 0

    # =========================================================================
    # 主定時器 callback（每 0.02s 執行一次）
    # =========================================================================
    def callback(self):
        self.step += 1

        # [1] 發送 controller service 請求（取 a, b）
        self.send_request()

        # [2] 發布 /timestep，告知 knee_controller 當前步數
        ts_msg = Int64()
        ts_msg.data = self.step
        self.timestep_publisher.publish(ts_msg)

        # [3] 把最新收到的 RL 修正量加到當前 step 的膝關節 CPG
        #     對應關係：[L0, L1, L2, R0, R1, R2] → leg[6, 5, 4, 1, 2, 3].osc[2]
        self._apply_knee_corrections(self.step)

        # [4] 間隔監控（保留原有 log 機制）
        current_time = time.time()
        interval = current_time - self.last_execution_time
        self.last_execution_time = current_time

        # [5] 驅動馬達（使用 step-1 的 CPG 值，維持原邏輯）
        walk(self.step - 1, self.a, self.b)

    # =========================================================================
    # 把 latest_corrections 累加到指定 step 的膝關節 CPG
    # =========================================================================
    def _apply_knee_corrections(self, step):
        # corrections 順序：[L0, L1, L2, R0, R1, R2]
        leg[6].osc[2].Y[step] += self.latest_corrections[0]  # L0
        leg[5].osc[2].Y[step] += self.latest_corrections[1]  # L1
        leg[4].osc[2].Y[step] += self.latest_corrections[2]  # L2
        leg[1].osc[2].Y[step] += self.latest_corrections[3]  # R0
        leg[2].osc[2].Y[step] += self.latest_corrections[4]  # R1
        leg[3].osc[2].Y[step] += self.latest_corrections[5]  # R2

    # =========================================================================
    # 與 controller service 通訊（完全保留原邏輯）
    # =========================================================================
    def send_request(self):
        if ((leg[1].osc[1].Y[self.step - 1] <= 0 and leg[1].osc[1].Y[self.step] > 0) or
            (leg[1].osc[1].Y[self.step - 1] >= 0 and leg[1].osc[1].Y[self.step] < 0)):
            self.req.if_control = 1
            self.req.get = change
            self.future = self.cli.call_async(self.req)
            self.future.add_done_callback(self.handle_response)
        else:
            self.req.if_control = 0
            self.req.get = change
            _ = self.cli.call_async(self.req)

    def handle_response(self, future):
        try:
            response = future.result()
            if (response.a != 666 and response.b != 666):
                self.a = response.a
                self.b = response.b
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    # =========================================================================
    # /knee_action topic 的訂閱 callback
    #   - 覆寫 latest_corrections 為最新值
    #   - 若 timestep 延遲超過閾值，印 warning log（延遲監控）
    # =========================================================================
    def knee_action_callback(self, msg: KneeAction):
        # 覆寫為最新收到的 6 維修正量
        self.latest_corrections = list(msg.corrections)

        # 延遲監控：比較當前 step 與 msg 中標示的 timestep
        delay = self.step - msg.timestep
        if delay > 2:
            self.get_logger().warn(
                f'knee_action delay: {delay} steps '
                f'(msg.timestep={msg.timestep}, current step={self.step})'
            )


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