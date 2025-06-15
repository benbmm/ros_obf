import time
import os
import numpy as np
import math
import sys
import threading
from queue import Queue

from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
from board import SCL, SDA
import busio

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from interfaces.msg import Output
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
servos = {}

SERVOS = [
    "R00", "R01", "R02", "R10", "R11", "R12", "R20", "R21", "R22",
    "L00", "L01", "L02", "L10", "L11", "L12", "L20", "L21", "L22"
]

Maxstep = 40000
NUM_SERVOS = 18
_count = 1000

# 是否開啟適應地形
adaption_mode = 1
change_mode = 0
change = 1

class OSC:
    def __init__(self):
        self.Y = np.zeros(_count + Maxstep)

class CPG:
    def __init__(self):
        self.osc = [OSC() for _ in range(5)]

leg = [CPG() for _ in range(7)]

# 新增：馬達控制隊列
motor_command_queue = Queue()
motor_thread_running = True

def motor_control_thread():
    """獨立的馬達控制執行緒"""
    global motor_thread_running
    while motor_thread_running:
        try:
            # 從隊列中取得馬達命令，設定超時避免阻塞
            command = motor_command_queue.get(timeout=0.1)
            if command is None:  # 結束信號
                break
            
            # 執行馬達控制命令
            motor_name, angle = command
            if motor_name in servos:
                try:
                    servos[motor_name].angle = angle
                except Exception as e:
                    print(f"馬達 {motor_name} 控制錯誤: {e}")
                    
        except:
            # 隊列為空時繼續等待
            continue

def walk(x, a, b):
    """優化的walk函數，使用非阻塞的馬達控制"""
    c = 1
    if not change:
        d = 1
    else:
        d = -1
    
    if adaption_mode == 1:
        e = 30
    else:
        e = 0
    
    # 距離牆太近原地旋轉
    if a == -2 and b == 2:
        d = 0
        a = -0.5
        b = 0.5
    elif a == 2 and b == -2:
        d = 0
        a = 0.5
        b = -0.5
    
    print(f"c={c}\td={d}\n")
    
    # 準備所有馬達命令
    motor_commands = []
    
    if x < 100:
        motor_commands = [
            ("R00", cpg_deg_change(0 * a)),
            ("R01", cpg_deg_change(0 * c) + e),
            ("R02", cpg_deg_change(0 * d) - e - 9),
            ("R10", cpg_deg_change(0 * a)),
            ("R11", cpg_deg_change(0 * c) + e),
            ("R12", cpg_deg_change(0 * d) - e),
            ("R20", cpg_deg_change(0 * a)),
            ("R21", cpg_deg_change(0 * c) + e),
            ("R22", cpg_deg_change(0 * d) - e - 1),
            ("L00", cpg_deg_change(0 * b)),
            ("L01", cpg_deg_change(0 * c) - e),
            ("L02", cpg_deg_change(0 * d) + e - 7),
            ("L10", cpg_deg_change(0 * b)),
            ("L11", cpg_deg_change(0 * c) - e),
            ("L12", cpg_deg_change(0 * d) + e - 7),
            ("L20", cpg_deg_change(0 * b)),
            ("L21", cpg_deg_change(0 * c) - e),
            ("L22", cpg_deg_change(0 * d) + e + 1)
        ]
    else:
        motor_commands = [
            ("R00", cpg_deg_change(leg[1].osc[1].Y[x] * a)),
            ("R01", cpg_deg_change(leg[1].osc[2].Y[x] * c) + e),
            ("R02", cpg_deg_change(leg[1].osc[2].Y[x] * d) - e - 9),
            ("R10", cpg_deg_change(leg[2].osc[1].Y[x] * a)),
            ("R11", cpg_deg_change(leg[2].osc[2].Y[x] * c) + e),
            ("R12", cpg_deg_change(leg[2].osc[2].Y[x] * d) - e),
            ("R20", cpg_deg_change(leg[3].osc[1].Y[x] * a)),
            ("R21", cpg_deg_change(leg[3].osc[2].Y[x] * c) + e),
            ("R22", cpg_deg_change(leg[3].osc[2].Y[x] * d) - e - 1),
            ("L00", cpg_deg_change(leg[6].osc[1].Y[x] * b)),
            ("L01", cpg_deg_change(leg[6].osc[2].Y[x] * c) - e),
            ("L02", cpg_deg_change(leg[6].osc[2].Y[x] * d) + e - 7),
            ("L10", cpg_deg_change(leg[5].osc[1].Y[x] * b)),
            ("L11", cpg_deg_change(leg[5].osc[2].Y[x] * c) - e),
            ("L12", cpg_deg_change(leg[5].osc[2].Y[x] * d) + e - 7),
            ("L20", cpg_deg_change(leg[4].osc[1].Y[x] * b)),
            ("L21", cpg_deg_change(leg[4].osc[2].Y[x] * c) - e),
            ("L22", cpg_deg_change(leg[4].osc[2].Y[x] * d) + e + 1)
        ]
    
    # 將所有馬達命令加入隊列（非阻塞）
    for command in motor_commands:
        try:
            motor_command_queue.put_nowait(command)
        except:
            # 隊列滿時跳過該命令
            print(f"警告：馬達命令隊列已滿，跳過命令 {command[0]}")
    
    print(f"x={x}\ta={a}\tb={b}\n")

def load_cpg():
    print("load cpg")
    file_paths = [
        f"/home/user/ros2_obf_ws/src/cpg/knee_high_5/YYout{i}{j}.txt" 
        for i in range(1, 7) for j in range(1, 4)
    ]
    files = [open(path, "r") for path in file_paths]

    for count in range(1, Maxstep + 1):
        for i in range(1, 7):
            for j in range(1, 4):
                file_index = (i - 1) * 3 + (j - 1)
                if files[file_index]:
                    value = float(files[file_index].readline().strip())
                    leg[i].osc[j].Y[count] = value

    for file in files:
        file.close()
    turn(Maxstep)

def cpg_deg_change(rad):
    deg = rad * (180 / math.pi)
    return deg + 90

def turn(num_count):
    for i in range(num_count + 1):
        leg[1].osc[3].Y[i] = -leg[1].osc[3].Y[i]
        leg[6].osc[3].Y[i] = -leg[6].osc[3].Y[i]
        leg[2].osc[3].Y[i] = -leg[2].osc[3].Y[i]
        leg[5].osc[3].Y[i] = -leg[5].osc[3].Y[i]

        if leg[2].osc[3].Y[i] < 0:
            leg[2].osc[3].Y[i] = 0
        if leg[5].osc[3].Y[i] < 0:
            leg[5].osc[3].Y[i] = 0

        for j in range(1, 7):
            if leg[j].osc[2].Y[i] < 0:
                leg[j].osc[2].Y[i] = 0
            if j <= 3:
                leg[j].osc[2].Y[i] = -leg[j].osc[2].Y[i]
                leg[j].osc[3].Y[i] = -leg[j].osc[3].Y[i]
            else:
                leg[j].osc[1].Y[i] = -leg[j].osc[1].Y[i]

def Servo_initialization():
    """伺服馬達初始化，增加錯誤處理"""
    try:
        for i, motor_name in enumerate(SERVOS[:9]):
            if i == 4:
                servo_motor = servo.Servo(pcaR.channels[9], min_pulse=600, max_pulse=2400)
            else:
                servo_motor = servo.Servo(pcaR.channels[i], min_pulse=600, max_pulse=2400)
            servo_motor.angle = 90
            servos[motor_name] = servo_motor

        for i, motor_name in enumerate(SERVOS[9:]):
            servo_motor = servo.Servo(pcaL.channels[i], min_pulse=600, max_pulse=2400)
            servo_motor.angle = 90
            servos[motor_name] = servo_motor
        
        print("伺服馬達初始化完成")
    except Exception as e:
        print(f"伺服馬達初始化錯誤: {e}")

class Servo(Node):
    def __init__(self, name):
        super().__init__(name)
        
        # 建立客戶端
        self.cli = self.create_client(Command, 'command')
        while not self.cli.wait_for_service(timeout_sec=0.05):
            self.get_logger().info('controller_service not available, waiting again...')
        
        if adaption_mode:
            self.cli_adaption = self.create_client(CommandAdaption, 'commandadaption')
            while not self.cli_adaption.wait_for_service(timeout_sec=0.05):
                self.get_logger().info('adaption_service not available, waiting again...')

        self.req = Command.Request()
        self.req_adaption = CommandAdaption.Request()
        
        # 使用較長的計時器間隔避免過度頻繁的回調
        timer_period = 0.03  # 從0.02增加到0.03
        self.timer = self.create_timer(timer_period, self.callback)
        
        self.last_execution_time = time.time()
        self.step = 0
        self.a = 1
        self.b = 1
        self.h = [0] * 8
        self.max_step = Maxstep
        self.count_controller_server = 0

    def callback(self):
        self.step += 1
        
        # 分離適應地形處理，避免阻塞主回調
        if adaption_mode and self.step > 115:
            # 使用非阻塞方式處理適應地形
            threading.Thread(target=self.handle_adaption, daemon=True).start()

        # 分離控制器服務處理
        threading.Thread(target=self.handle_controller, daemon=True).start()
        
        # 執行步態控制（非阻塞）
        current_time = time.time()
        interval = current_time - self.last_execution_time
        self.last_execution_time = current_time
        
        # 在獨立執行緒中執行walk函數
        threading.Thread(target=walk, args=(self.step-1, self.a, self.b), daemon=True).start()

    def handle_adaption(self):
        """處理適應地形服務請求"""
        try:
            self.send_request_adaption()
        except Exception as e:
            self.get_logger().error(f'Adaption handling error: {e}')

    def handle_controller(self):
        """處理控制器服務請求"""
        try:
            self.send_request()
        except Exception as e:
            self.get_logger().error(f'Controller handling error: {e}')

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
            if response.a != 666 and response.b != 666:
                self.a = response.a
                self.b = response.b
            self.get_logger().info(f"a={self.a},b={self.b},step={self.step}")
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def send_request_adaption(self):
        self.req_adaption.step = self.step
        self.future_adaption = self.cli_adaption.call_async(self.req_adaption)
        self.future_adaption.add_done_callback(self.handle_response_adaption)

    def handle_response_adaption(self, future_adaption):
        try:
            response_adaptio = future_adaption.result()
            self.h[1] = response_adaptio.h1
            self.h[2] = response_adaptio.h2
            self.h[3] = response_adaptio.h3
            self.h[4] = response_adaptio.h4
            self.h[5] = response_adaptio.h5
            self.h[6] = response_adaptio.h6
            
            if change_mode:
                global change
                change = response_adaptio.change
                
            self.get_logger().info(f'h1: {response_adaptio.h1}\th2: {response_adaptio.h2}\th3: {response_adaptio.h3}\th4: {response_adaptio.h4}\th5: {response_adaptio.h5}\th6: {response_adaptio.h6}\nchange:{response_adaptio.change}\tstep:{self.step}\n')
            
            if self.step > 0:
                for i in range(1, 7):
                    if self.h[i] != leg[i].osc[2].Y[self.step]:
                        leg[i].osc[2].Y[self.step] = self.h[i]
                        
        except Exception as e:
            self.get_logger().error(f'Service_adaption call failed: {e}')

def main():
    global motor_thread_running
    
    # 載入CPG和初始化
    load_cpg()
    Servo_initialization()
    
    # 啟動馬達控制執行緒
    motor_thread = threading.Thread(target=motor_control_thread, daemon=True)
    motor_thread.start()
    
    # 初始化ROS2
    rclpy.init()
    node = Servo("servo")
    
    # 使用多執行緒執行器
    executor = MultiThreadedExecutor(num_threads=4)
    
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        # 清理資源
        motor_thread_running = False
        motor_command_queue.put(None)  # 發送結束信號
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()