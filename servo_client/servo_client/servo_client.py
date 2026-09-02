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
from std_msgs.msg import Bool               # /adaptation_ready 信號
from interfaces.msg import StepCPG            # 發布 /timestep 用
from interfaces.msg import Output           # 字符串消息类型
from interfaces.msg import KneeAction       # 訂閱 /knee_action 用
from interfaces.srv import Command
from rclpy.qos import QoSProfile, DurabilityPolicy

from rclpy.qos import QoSProfile, DurabilityPolicy

adaptation_ready_qos = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL
)

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

# 是否開啟適應地形（保留原語義：控制 walk() 裡的 e=30 抬高機身）
adaption_mode = 1
# change 固定為 1，不再動態切換（原本的 adaption service 已改為 knee_controller 節點）
#change是指在兩種模式切換:適應地形的時候踝關節固定，適應完則開啟踝關節，在controller node 中模糊控制器參數也會隨之切換
#當change_mode=1 代表change值會隨著adaption node回傳值變化。=0時則始終不變，因此不會切換控制器，踝關節也維持固定角度
change_mode=0
# 當adaption node偵測到姿態變化，回傳chang=1，servo node傳給controller node
change = 1

class OSC:
    def __init__(self):
        self.Y = np.zeros(_count + Maxstep)

class CPG:
    def __init__(self):
        # 初始化CPG的數據
        self.osc = [OSC() for _ in range(5)]  # osc有5個元素

leg = [CPG() for _ in range(7)]  # 6個腿+1個（從1開始）

def load_cpg():
    print("load cpg")
    file_paths = [
        f"/home/user/ros2_obf_ws/src/cpg/fixed_cpg_1.4/YYout{i}{j}.txt" for i in range(1, 7) for j in range(1, 4)
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
        
    # 對膝關節 (osc[2]) 和踝關節 (osc[3]) 套用 gain
    knee_ankle_gain = 1.4
    for i in range(1, 7):
        for count in range(Maxstep + 1):
            leg[i].osc[2].Y[count] *= knee_ankle_gain
            leg[i].osc[3].Y[count] *= knee_ankle_gain
    
    
    turn(Maxstep)

def cpg_deg_change(rad):
    deg = rad * (180 / math.pi)
    return deg + 90

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
        #馬達型號不同導致方向不同
        leg[3].osc[1].Y[i]=-leg[3].osc[1].Y[i]

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

        # ---- Service client：與 controller 節點通訊 ----
        self.cli = self.create_client(Command, 'command')
        while not self.cli.wait_for_service(timeout_sec=0.05):
            self.get_logger().info('controller_service not available, waiting again...')
        self.req = Command.Request()

        # ---- Publisher：發布 /timestep 給 knee_controller ----
        self.timestep_publisher = self.create_publisher(StepCPG, 'timestep', 1)

        # ---- Subscriber：訂閱 /knee_action ----
        self.knee_action_sub = self.create_subscription(
            KneeAction,
            'knee_action',
            self.knee_action_callback,
            1
        )

        # ---- Subscriber：訂閱 /adaptation_ready ----
        # adaptation warmup 完成且收到第一筆 IMU 後發布一次 True；
        # 收到後才允許 callback() 推進 step。
        self.create_subscription(
            Bool,
            'adaptation_ready',
            self.adaptation_ready_callback,
            adaptation_ready_qos
        )

        # ---- 狀態變數 ----
        self.step = 0
        self.a = 1
        self.b = 1
        self.latest_corrections = [0.0] * 6

        self.current_a = 1
        self.current_b = 1
        self.current_corrections = [0.0] * 6
        self.max_step = Maxstep
        self.count_controller_server = 0

        self.is_waiting = True

        # 修正量去重用：記錄上一次成功套用 _apply_knee_corrections() 的 step。
        # phase 1 / phase 2 兩個動作 tick 共用同一個 execute_step，
        # 避免同一個 step 的 CPG 表被重複疊加修正量（見 _do_action_tick）。
        self._last_corrected_step = -1

        # adaptation 就緒旗標：收到第一筆 knee_action 才設為 True。
        # 在此之前 callback() 不推進任何 step，確保 warmup 與 IMU 都已完成。
        self._adaptation_ready: bool = False

        self.send_timestamps = {}

        # ==============================================================
        # 新增：馬達角度與速率限制器狀態
        # ==============================================================
        self.current_angles = {motor: 90.0 for motor in SERVOS}
        self.max_deg_change = 0.17 * (180.0 / math.pi)  # 0.17 rad 轉換為度數 (約 9.74度)

        # ==============================================================
        # 預備階段：node 建立時先讓馬達移動到預備姿勢，並阻塞等待 2 秒
        # 讓馬達確實到位。此階段不遞增 self.step、不發布 /timestep、
        # 不呼叫 send_request()，完全不影響後續正式行走階段的 step 計數。
        # 注意：time.sleep() 期間 ROS2 executor 無法處理任何 callback，
        # 但因為這是在 __init__ 階段、timer 尚未建立、也還沒 rclpy.spin()，
        # 所以不會有其他 callback 被卡住的問題。
        # ==============================================================
        self.walk_prepare()
        time.sleep(2.0)

        # ---- 定時器（0.01s, 100Hz）----
        # 每 0.01s 觸發一次，以 self.phase 在三個階段輪替：
        # phase 0: 請求 tick（step 遞增、發布 /timestep、呼叫 send_request，不走馬達）
        # phase 1: 動作 tick 1（套用最新 latest_corrections / a / b，呼叫 walk）
        # phase 2: 動作 tick 2（再次使用同一批修正量，呼叫 walk）
        # 時序：發送推論請求 0.01s -> 新修正生效 0.01s -> 再走一次 0.01s，共 0.03s 一週期
        timer_period = 0.01
        self.phase = 0
        self.action_hold_ticks = 2

        self.timer = self.create_timer(timer_period, self.callback)
        self.last_execution_time = time.time()
        self.dt_list = []
        self.rtt_list = []

    # =========================================================================
    # 速率限制器輔助函數
    # =========================================================================
    def set_servo_angle_immediate(self, motor_name, target_angle):
        """不經過速率限制，直接將馬達移動到目標角度，並同步更新內部追蹤狀態。
        僅供 walk_prepare() 在預備階段使用，讓馬達一次到位；
        正式行走階段一律使用 set_servo_angle()（含速率限制）。"""
        safe_angle = np.clip(target_angle, 0.0, 180.0)
        self.current_angles[motor_name] = safe_angle
        servos[motor_name].angle = safe_angle

    def set_servo_angle(self, motor_name, target_angle):
        """將計算出的目標角度經過速率限制後，寫入硬體並更新狀態"""
        current_angle = self.current_angles[motor_name]
        
        # 限制單次變化的最大差值
        diff = target_angle - current_angle
        
        limited_diff = np.clip(diff, -self.max_deg_change, self.max_deg_change)
        #if(motor_name=="R01" or motor_name=="L01"):
        #    self.get_logger().info(f'{motor_name}diff={diff}limited_diff={limited_diff}')
        
        # 計算最終安全角度，並確保不超出伺服馬達物理限制 (0~180度)
        safe_angle = current_angle + limited_diff
        safe_angle = np.clip(safe_angle, 32.70422, 147.2958)#90-180/pi,90+180/pi
        
        # 更新內部追蹤狀態與實體馬達
        self.current_angles[motor_name] = safe_angle
        servos[motor_name].angle = safe_angle

    # =========================================================================
    # 整合了速率限制的 walk 函數 (移入 class 內)
    # =========================================================================
    def walk(self, x, a, b):
        a=0
        b=0
        c = 1
        if (not change):
            d = 1
        else:
            d = -1
        
        if (adaption_mode == 1):
            e = 34.377
        else:
            e = 0
        
        #距離牆太近原地旋轉
        if (a == -2 and b == 2):
            d = 0
            a = -0.5
            b = 0.5
        elif (a == 2 and b == -2):
            d = 0
            a = 0.5
            b = -0.5

        # 改用 self.set_servo_angle 套用速率限制
        self.set_servo_angle("R00", cpg_deg_change((leg[1].osc[1].Y[x] * a)))
        self.set_servo_angle("R01", cpg_deg_change((leg[1].osc[2].Y[x] * c)) + e-10)
        self.set_servo_angle("R02", cpg_deg_change((leg[1].osc[2].Y[x] * d)) - e)
        self.set_servo_angle("R10", cpg_deg_change((leg[2].osc[1].Y[x] * a))+3.5)
        self.set_servo_angle("R11", cpg_deg_change((leg[2].osc[2].Y[x] * c)) + e)
        self.set_servo_angle("R12", cpg_deg_change((leg[2].osc[2].Y[x] * d)) - e)
        self.set_servo_angle("R20", cpg_deg_change((leg[3].osc[1].Y[x] * a)))
        self.set_servo_angle("R21", cpg_deg_change((leg[3].osc[2].Y[x] * c)) + e)
        self.set_servo_angle("R22", cpg_deg_change((leg[3].osc[2].Y[x] * d)) - e)

        self.set_servo_angle("L00", cpg_deg_change((leg[6].osc[1].Y[x] * b)))
        self.set_servo_angle("L01", cpg_deg_change((leg[6].osc[2].Y[x] * c)) - e)
        self.set_servo_angle("L02", cpg_deg_change((leg[6].osc[2].Y[x] * d)) + e+5)
        self.set_servo_angle("L10", cpg_deg_change((leg[5].osc[1].Y[x] * b)))
        self.set_servo_angle("L11", cpg_deg_change((leg[5].osc[2].Y[x] * c)) - e+5)
        self.set_servo_angle("L12", cpg_deg_change((leg[5].osc[2].Y[x] * d)) + e+5)
        self.set_servo_angle("L20", cpg_deg_change((leg[4].osc[1].Y[x] * b)))
        self.set_servo_angle("L21", cpg_deg_change((leg[4].osc[2].Y[x] * c)) - e-5)
        self.set_servo_angle("L22", cpg_deg_change((leg[4].osc[2].Y[x] * d)) + e)

    # =========================================================================
    # 預備動作：讓機器人從初始 90 度姿態，移動到行走前的固定預備姿勢。
    # 內容取自原本 walk() 裡 `x < start_walking_step` 的固定角度分支，
    # 但此函式只在節點啟動時呼叫一次，不帶 step 概念、不查 CPG 表，
    # 也不會遞增 self.step 或發布 /timestep，純粹只是把馬達移動到位。
    # =========================================================================
    def walk_prepare(self):
        if (adaption_mode == 1):
            e = 34.377
            #e=0
        else:
            e = 0

        self.set_servo_angle_immediate("R00", cpg_deg_change(0))
        self.set_servo_angle_immediate("R01", cpg_deg_change(0) + e-10)
        self.set_servo_angle_immediate("R02", cpg_deg_change(0) - e)
        self.set_servo_angle_immediate("R10", cpg_deg_change(0)+3.5)
        self.set_servo_angle_immediate("R11", cpg_deg_change(0) + e)
        self.set_servo_angle_immediate("R12", cpg_deg_change(0) - e)
        self.set_servo_angle_immediate("R20", cpg_deg_change(0))
        self.set_servo_angle_immediate("R21", cpg_deg_change(0) + e)
        self.set_servo_angle_immediate("R22", cpg_deg_change(0) - e)

        self.set_servo_angle_immediate("L00", cpg_deg_change(0))
        self.set_servo_angle_immediate("L01", cpg_deg_change(0) - e)
        self.set_servo_angle_immediate("L02", cpg_deg_change(0) + e+5)
        self.set_servo_angle_immediate("L10", cpg_deg_change(0))
        self.set_servo_angle_immediate("L11", cpg_deg_change(0) - e+5)
        self.set_servo_angle_immediate("L12", cpg_deg_change(0) + e+5)
        self.set_servo_angle_immediate("L20", cpg_deg_change(0))
        self.set_servo_angle_immediate("L21", cpg_deg_change(0) - e-5)
        self.set_servo_angle_immediate("L22", cpg_deg_change(0) + e)

    # =========================================================================
    # 主定時器 callback（每 0.01s 執行一次，三 phase 輪替）
    #
    # phase 0（請求 tick）：step 遞增、發布 /timestep（觸發 knee_controller 推論）、
    #                        呼叫 send_request()，本 tick 不走馬達。
    # phase 1（動作 tick 1）：在請求 tick 後 0.01s，套用目前最新的
    #                          latest_corrections / a / b 並呼叫 walk()。
    # phase 2（動作 tick 2）：再過 0.01s，使用相同修正量再次呼叫 walk()。
    #
    # 時序：發請求(0.01s) -> 新修正生效走馬達(0.01s) -> 再走一次(0.01s)，共 0.03s 一週期。
    # 全程不使用任何阻塞等待（如 time.sleep），讓 ROS2 executor 在 tick 之間
    # 仍能正常處理 knee_action_callback。
    # =========================================================================
    def callback(self):

        if self.step >= self.max_step:
            self.get_logger().info(f'已達到最大步數 ({self.max_step} 步)！停止發送指令，保持當前姿勢。')
            self.timer.cancel()
            return

        if self.count_publishers('knee_action') == 0 or self.count_subscribers('timestep') == 0:
            if self.is_waiting:
                self.get_logger().info('等待 knee_action 節點完成連線（publisher + timestep subscriber）中...')
                self.is_waiting = False
            return

        if not self.is_waiting:
            self.get_logger().info('已偵測到 knee_action 節點連線，正式開始執行！')
            self.is_waiting = True
            self.last_execution_time = time.time()
            self.dt_list = []

        # adaptation 尚未就緒（warmup 未完成或 IMU 尚未收到第一筆資料）：
        # 不推進任何 step，靜待 /adaptation_ready 信號。
        if not self._adaptation_ready:
            return

        if self.phase == 0:
            self._do_request_tick()
            self.phase = 1
        else:
            self._do_action_tick()

            if self.phase >= self.action_hold_ticks:
                self.phase = 0
            else:
                self.phase += 1

    def _do_request_tick(self):
        """請求 tick：量測完整週期 dt（與兩個動作 tick 合計約 0.03s）、
        step 遞增、發布 /timestep、呼叫 send_request()。本 tick 不走馬達。"""
        current_time = time.time()
        dt = current_time - self.last_execution_time
        self.last_execution_time = current_time

        if dt > 0 and dt < 1.0:
            self.dt_list.append(dt)

        if len(self.dt_list) >= 50:
            avg_dt = sum(self.dt_list) / len(self.dt_list)
            avg_hz = 1.0 / avg_dt
            self.get_logger().info(f'平均執行頻率: {avg_hz:.2f} Hz')
            self.dt_list.clear()

        # ---------------------------------------------------------
        # 🚀 發送請求，觸發這一步的推論
        # ---------------------------------------------------------
        self.step += 1

        ts_msg = StepCPG()
        ts_msg.timestep = self.step
        safe_step = self.step % Maxstep
        if safe_step == 0: safe_step = Maxstep

        ts_msg.cpg_value = float(leg[1].osc[1].Y[safe_step])

        self.send_timestamps[self.step] = time.time()

        self.timestep_publisher.publish(ts_msg)

        self.send_request()
        #self.get_logger().info(f"send_request {self.step}")

    def _do_action_tick(self):
        """動作 tick：固定在請求 tick 之後執行（phase 1 或 phase 2），
        不論 knee_action 是否已更新，直接用目前 self.latest_corrections / a / b 套用馬達動作。
        同一批修正量會在 phase 1 與 phase 2 各執行一次，共持續 0.02s。

        注意：_apply_knee_corrections() 是用 += 疊加到 CPG 原始資料表上，
        若同一個 execute_step 重複呼叫會疊加兩次修正量。phase 1 / phase 2
        共用同一個 execute_step（self.step 只在 phase 0 遞增），因此這裡用
        self._last_corrected_step 確保每個 step 只疊加一次：
        phase 1 套用修正並寫入 CPG 表，phase 2 偵測到同一個 step 後跳過
        疊加，只重新呼叫 walk() 用同一批已修正完成的角度再走一次。"""
        execute_step = self.step

        if execute_step != self._last_corrected_step:
            self._apply_knee_corrections(execute_step, self.latest_corrections)
            self._last_corrected_step = execute_step

        self.walk(execute_step, self.a, self.b)
 
    def _apply_knee_corrections(self, step, corrections):
        # corrections 順序：[L0, L1, L2, R0, R1, R2]
        #active_idx = (step // 250) % 6
        #corrections = [0.0] * 6
        #corrections[active_idx] = 0.5
        #self.get_logger().info(f'corrections={corrections}')
        leg[6].osc[2].Y[step] += corrections[0]  # L0
        leg[5].osc[2].Y[step] += corrections[1]  # L1
        leg[4].osc[2].Y[step] += corrections[2]  # L2
        leg[1].osc[2].Y[step] -= corrections[3]  # R0
        leg[2].osc[2].Y[step] -= corrections[4]  # R1
        leg[3].osc[2].Y[step] -= corrections[5]  # R2

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

    def adaptation_ready_callback(self, msg: Bool):
        if msg.data and not self._adaptation_ready:
            self._adaptation_ready = True
            self.last_execution_time = time.time()
            self.get_logger().info(
                'adaptation 已就緒（warmup 完成 + IMU 有資料），開始正式推進 step！'
            )

    def knee_action_callback(self, msg: KneeAction):
        self.latest_corrections = list(msg.corrections)
        recv_time = time.time()
        send_time = self.send_timestamps.pop(msg.timestep, None)
        if send_time is not None:
            rtt = recv_time - send_time
            if rtt > 0 and rtt < 1.0:
                self.rtt_list.append(rtt)

            if len(self.rtt_list) >= 50:
                avg_rtt = sum(self.rtt_list) / len(self.rtt_list)
                self.get_logger().info(f'平均 RTT: {avg_rtt*1000:.2f} ms')
                self.rtt_list.clear()

        delay = self.step - msg.timestep
        if delay > 0:
            self.get_logger().warn(
                f'knee_action delay: {delay} steps '
                f'(msg.timestep={msg.timestep}, current step={self.step})'
            )

        # ---- 提早套用（Early Apply）----
        # 條件：phase == 1（phase 0 已發出請求、phase 1 timer 尚未到期）
        #        且收到的正好是當前 step 的回應
        # _last_corrected_step 確保後續 phase 1 / phase 2 的 _do_action_tick
        # 不會再重複呼叫 _apply_knee_corrections，但仍會正常呼叫 walk()。
        if self.phase == 1:
            if msg.timestep == self.step:
                self._apply_knee_corrections(self.step, self.latest_corrections)
                self._last_corrected_step = self.step
                self.walk(self.step, self.a, self.b)
                self.get_logger().debug(
                    f'[early apply] step={self.step} 修正量提早套用並執行馬達'
                )
            else:
                # phase 1 等待期間收到不屬於當前 step 的回應
                # （若 delay 警告已出現，表示是舊的延遲訊號）
                self.get_logger().warn(
                    f'[early apply 跳過] phase=1 但 msg.timestep={msg.timestep} '
                    f'!= self.step={self.step}，修正量將於 phase 1 timer 到期後套用'
                )


def main():                                       
    load_cpg()
    Servo_initialization()

    rclpy.init()              
    node = Servo("servo")  

    rclpy.spin(node)

    node.destroy_node()                            
    rclpy.shutdown()                               

if __name__ == '__main__':
    main()