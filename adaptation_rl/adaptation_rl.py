"""
膝關節控制器 ROS2 節點（主節點）

功能:
    訂閱 /imu/data 與 /timestep，每收到一次 /timestep 即執行一次推論，
    將 6 維膝關節修正量（rl_corrections, 範圍 0~0.7）發布至 /knee_action。

本檔案整合以下邏輯:
    1. CPG 資料讀取（純浮點數一行一數）
    2. 四元數 → roll/pitch 轉換 + 6 維 IMU 狀態展開
    3. TensorRT FP16 engine 載入與推論（原為 PyTorch 模型推論）
    4. ROS2 節點主體（兩個 callback + 推論主迴圈）

模型架構說明（PPOModel / transformer_models_priv，Cross-Attention Transformer
+ Privileged State Estimator，取代舊版 HexaPrivStudentModel）:
    - 模型輸入是 328 維 = 320 (SEQ_LEN*FEATURE_DIM 歷史觀測) + 8 (PRIV_DIM，
      priv_gt 佔位)。真實機器人沒有 feet_clearance / base_velocity 等特權感測器，
      匯出 ONNX 時已將 priv_mix_alpha 固定設為 1.0，代表 actor 完全依賴
      PrivEstimator 估計出來的 priv_pred，因此 priv_gt 這 8 維填什麼數值都
      不影響 actor 輸出，這裡固定填 0。
    - 已轉換為 TensorRT FP16 engine（student_policy_fp16.engine）
    - 輸入: 展平的觀察序列 + priv_gt 佔位 (1, TOTAL_INPUT_DIM) = (1, 328)
      （與訓練/PyTorch 版本一致，TensorRT engine 內部對應同一份計算圖）
    - 輸入正規化: 訓練時的 skrl RunningStandardScaler（對全部 328 維一起做）
      已在 export_onnx.py 階段烘焙進 TensorRT engine 的權重中
      （ONNX 匯出時一併固化，不需要額外處理）
    - 推論: self.trt_runner.infer(obs_flat)，取代原本的
      student.act({"states": obs_flat}, role="policy")

效能改善（中、高影響）:
    [高] obs_history 使用環形指標（circular buffer）取代 torch.roll
         每步只寫入一個 index，不再分配新的 (100, 16) tensor
    [高] imu_callback 改存 Python tuple，消除高頻 GPU 操作
         GPU 操作集中在 timestep_callback（控制頻率），而非 IMU 頻率
    [高] imu_callback 移除 6 次 flush，統一在 timestep_callback 執行
    [中] calculate_imu_states 改回傳 Python tuple，不在函式內建 tensor
    [中] current_obs 改為預配置 GPU buffer（_obs_buf）+ slice 填值
         取代 torch.cat（每步都分配新 tensor）
    [中] _flat_obs_dim 預先計算，避免每步重複乘法

第二輪效能改善:
    [高] 移除 timestep_callback 中重複的 GPU→CPU 同步
         原本 rl_corrections[i].item() 對同一份資料又同步 6 次，
         現直接重複使用第一次 .cpu().tolist() 的結果（data_list）
    [中] imu 9 維資料（imu_vals + ang_vel）改用預先配置好的
         pinned-memory CPU staging buffer 重複填值，
         取代每步 torch.tensor(...) 重新配置記憶體
    [中] quaternion → roll/pitch 改用手寫數學公式取代 scipy Rotation，
         省去物件建構與通用矩陣運算開銷（公式與 scipy as_euler("xyz") 等價）
    [高] 檔案 I/O 改為「方案 C」：預先配置 numpy 陣列（上限 40000 筆）作為
         記憶體緩衝區，控制迴圈中只做記憶體寫入，不做任何磁碟 I/O；
         超過 40000 筆後不再記錄（不覆寫，直接停止寫入緩衝區）；
         結束時（destroy_node）才一次性寫出檔案，
         徹底移除控制迴圈中的 write()/flush() 阻塞

第三輪效能改善（TensorRT FP16 推論）:
    [高] 推論引擎由 PyTorch (PPOModel.act) 改為
         TensorRT FP16 engine（student_policy_fp16.engine）。
         轉換流程: PyTorch -> ONNX (dynamo=True exporter) -> trtexec FP16 engine，
         已用 200 組隨機輸入驗證數值精度（換算到實際輸出範圍 0~0.7 後，
         最大誤差約 4.65e-3，平均誤差約 6.5e-4），並在 Jetson Orin Nano 上
         實測 GPU 推論時間平均 0.34ms（trtexec 量測），相較 PyTorch 直接推論
         有顯著加速，且遠低於控制迴圈週期（0.04s），不構成瓶頸。
    [說明] TensorRT 10.x 使用 execute_async_v3 API，透過 set_tensor_address
         以名稱（而非索引）綁定輸入/輸出記憶體位址；輸入/輸出記憶體直接使用
         PyTorch CUDA tensor（.data_ptr()），不額外引入 pycuda 依賴，
         與既有的 obs_history / _obs_buf GPU pipeline 無縫接軌。
    [固定] 輸入 shape 固定為 (1, 328)，與 TensorRT engine 建置時一致，
         不支援動態 batch（與實際部署情境相符，不需要這個彈性）。
"""

import os
import math

import numpy as np
import torch
import tensorrt as trt

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu
from interfaces.msg import KneeAction
from interfaces.msg import StepCPG
import time
from rclpy.qos import QoSProfile, DurabilityPolicy

# 本專案
from adaptation_rl.config import NodeConfig

from rclpy.qos import QoSProfile, DurabilityPolicy

adaptation_ready_qos = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL
)


# =============================================================================
# TensorRT engine 路徑（由 export_onnx.py + convert_trt.sh 產生）
# 已改為在 NodeConfig.ENGINE_PATH 中設定（config.py），不在此處寫死。
# =============================================================================


# =============================================================================
# 模組層級常數
# =============================================================================

_SQRT_HALF: float = math.sqrt(0.5)


# =============================================================================
# IMU 工具
# =============================================================================

def quat_to_roll_pitch(x: float, y: float, z: float, w: float) -> tuple:
    """
    四元數 → (roll, pitch)，公式與
    scipy.spatial.transform.Rotation.from_quat([x, y, z, w]).as_euler("xyz")
    完全等價（intrinsic XYZ 慣例），但省去物件建構與通用矩陣運算開銷。

    Returns:
        (roll, pitch): 單位為弧度
    """
    # roll (x-axis rotation)
    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    # pitch (y-axis rotation)
    t2 = 2.0 * (w * y - z * x)
    t2 = 1.0 if t2 > 1.0 else (-1.0 if t2 < -1.0 else t2)  # clamp 避免 domain error
    pitch = math.asin(t2)

    return roll, pitch


def calculate_imu_states(roll: float, pitch: float) -> tuple:
    """
    將 roll/pitch 展開為 6 維 IMU 狀態（與訓練環境 _calculate_imu_states 完全一致）。

    改為回傳 Python tuple（不在此函式內建 tensor），
    由 timestep_callback 統一完成 CPU → GPU 搬運，減少不必要的 tensor 配置。

    Returns:
        (i0, i1, i2, i3, i4, i5): 6 個 Python float
    """
    return (
        (-pitch - roll) * _SQRT_HALF,   # [0]
        -roll,                           # [1]
        (pitch - roll) * _SQRT_HALF,    # [2]
        (pitch + roll) * _SQRT_HALF,    # [3]
        roll,                            # [4]
        (-pitch + roll) * _SQRT_HALF,   # [5]
    )


# =============================================================================
# TensorRT engine 載入與推論
# =============================================================================

class TRTEngineRunner:
    """
    包裝 TensorRT engine 的載入與推論。

    使用 TensorRT 10.x 的 execute_async_v3 API（execute_async_v2 在這個版本已棄用），
    透過 set_tensor_address 以名稱綁定輸入/輸出記憶體位址。

    輸入/輸出記憶體直接使用 PyTorch CUDA tensor（.data_ptr() 取得 GPU 位址），
    不額外引入 pycuda 依賴，方便跟既有的 obs_history / _obs_buf GPU pipeline
    無縫接軌（obs_flat 本身就已經是 GPU tensor，可直接傳入 infer()）。

    固定輸入 shape = (1, flat_obs_dim)，與 export_onnx.py / convert_trt.sh
    建置時使用的 shape 一致，不支援動態 batch。
    """

    def __init__(self, engine_path: str, device: torch.device):
        if not os.path.exists(engine_path):
            raise FileNotFoundError(f"找不到 TensorRT engine 檔案: {engine_path}")

        self.device = device
        self.logger = trt.Logger(trt.Logger.WARNING)
        self.runtime = trt.Runtime(self.logger)

        with open(engine_path, "rb") as f:
            engine_data = f.read()
        self.engine = self.runtime.deserialize_cuda_engine(engine_data)
        if self.engine is None:
            raise RuntimeError(
                f"無法反序列化 TensorRT engine: {engine_path}"
                "（檔案可能損壞，或與目前安裝的 TensorRT 版本不相容）"
            )

        self.context = self.engine.create_execution_context()

        # 找出唯一的輸入/輸出 tensor 名稱（對應 export_onnx.py 的
        # input_names=["states"], output_names=["mean_actions"]）
        self.input_names = []
        self.output_names = []
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            mode = self.engine.get_tensor_mode(name)
            if mode == trt.TensorIOMode.INPUT:
                self.input_names.append(name)
            else:
                self.output_names.append(name)

        if len(self.input_names) != 1 or len(self.output_names) != 1:
            raise RuntimeError(
                f"預期 1 個輸入、1 個輸出，但找到輸入={self.input_names}, "
                f"輸出={self.output_names}。請確認 engine 是否對應預期的單輸入單輸出模型。"
            )

        self.input_name = self.input_names[0]
        self.output_name = self.output_names[0]
        self.output_shape = tuple(self.engine.get_tensor_shape(self.output_name))

        # 預先配置固定大小的輸出 buffer（GPU tensor），重複使用、不每次重新配置
        self._output_buf = torch.empty(
            self.output_shape, dtype=torch.float32, device=self.device
        ).contiguous()

        # 使用 PyTorch 目前的 CUDA stream，不額外建立獨立 stream，
        # 確保跟其他 GPU 操作（obs_history 的寫入/讀取）排在同一個執行序列上
        self.stream = torch.cuda.current_stream(device=self.device)

    def infer(self, input_tensor: torch.Tensor) -> torch.Tensor:
        """
        input_tensor: GPU 上的 float32 tensor, shape=(1, flat_obs_dim)，必須 contiguous。
        回傳: GPU 上的 float32 tensor, shape=(1, action_dim)
              （engine 內部以 FP16 計算，輸出已轉為 float32，可直接接續後續
              clamp / 線性映射等運算，不需要額外型別轉換）
        """
        self.context.set_tensor_address(self.input_name, input_tensor.data_ptr())
        self.context.set_tensor_address(self.output_name, self._output_buf.data_ptr())

        ok = self.context.execute_async_v3(stream_handle=self.stream.cuda_stream)
        if not ok:
            raise RuntimeError("TensorRT execute_async_v3 執行失敗")

        return self._output_buf


# =============================================================================
# ROS2 節點主體
# =============================================================================

class KneeControllerNode(Node):
    """
    單執行緒 ROS2 節點。
    - /imu/data  callback: 高頻，僅快取 roll/pitch/angular_velocity（Python tuple），不做推論。
    - /timestep  callback: 觸發完整推論流程，發布 /knee_action。
    """

    def __init__(self):
        super().__init__("knee_controller")

        self.cfg = NodeConfig()
        self.device = torch.device(self.cfg.DEVICE)
        self.get_logger().info(f"Using device: {self.device}")

        # ---- 載入 TensorRT engine（取代原本的 PyTorch Student 模型）----
        self.get_logger().info(f"Loading TensorRT engine: {self.cfg.ENGINE_PATH}")
        self.trt_runner = TRTEngineRunner(self.cfg.ENGINE_PATH, self.device)
        self.get_logger().info("TensorRT engine loaded.")

        # ---- 預計算常數 ----
        # 預先計算，避免 timestep_callback 每步重複乘法
        self._flat_obs_dim: int = self.cfg.SEQ_LEN * self.cfg.FEATURE_DIM  # 320 (歷史觀測部分)
        self._total_input_dim: int = self.cfg.TOTAL_INPUT_DIM  # 328 (320 歷史 + 8 priv_gt 佔位)

        # ---- 模型輸入 buffer（直接建在 GPU，固定 328 維）----
        # layout: [0:320] 歷史觀測 (每步覆寫) | [320:328] priv_gt 佔位 (固定為 0，不再覆寫)
        # priv_mix_alpha 在匯出 ONNX 時已固定為 1.0，actor 完全依賴 priv_pred，
        # 因此這 8 維填 0 不影響推論結果，只需初始化一次。
        self._model_input_buf = torch.zeros(
            (1, self._total_input_dim), dtype=torch.float32, device=self.device
        )

        # ---- obs_history（環形 buffer，直接建在 GPU）----
        # 使用 _buf_head 指標取代 torch.roll，每步只寫入一個 index
        self.obs_history = torch.zeros(
            self.cfg.OBS_BUFFER_LENGTH, self.cfg.FEATURE_DIM,
            dtype=torch.float32, device=self.device,
        )
        # _buf_head: 下一次寫入的位置（同時也是目前最舊資料的位置）
        self._buf_head: int = 0

        # 取樣偏移量：從 _buf_head 往後數的邏輯偏移 [4, 9, 14, ..., 99]
        # 對應 stride-5 取樣的 20 個時間步（最舊 → 最新）
        self._subsample_offsets = torch.tensor(
            [self.cfg.OBS_SUBSAMPLE_STRIDE * (i + 1) - 1
             for i in range(self.cfg.SEQ_LEN)],
            dtype=torch.long,
            device=self.device,
        )

        # ---- 預配置觀測組裝 buffer（直接建在 GPU）----
        # 取代每步 torch.cat，改為 in-place slice 填值
        # layout: [0] cpg | [1:7] last_action | [7:13] imu_states | [13:16] ang_vel
        self._obs_buf = torch.zeros(
            self.cfg.FEATURE_DIM, dtype=torch.float32, device=self.device
        )

        # ---- last_action（GPU）----
        self.last_action = torch.zeros(
            self.cfg.ACTION_DIM, dtype=torch.float32, device=self.device
        )

        # ---- IMU staging buffer（CPU pinned memory，重複使用）----
        # 取代每步 torch.tensor((*imu_vals, *self._ang_vel), ...) 重新配置記憶體，
        # pinned memory 可讓 H2D copy 以非同步方式進行（non_blocking=True）。
        # 只在 device 為 cuda 時申請 pinned memory；CPU-only 環境 pin_memory 無意義。
        self._imu_staging = torch.zeros(9, dtype=torch.float32)
        if self.device.type == "cuda":
            self._imu_staging = self._imu_staging.pin_memory()

        # ---- IMU 快取（Python tuple，無 GPU 操作）----
        # imu_callback 高頻觸發，改存 Python tuple 避免每次 GPU 配置
        # GPU tensor 在 timestep_callback 一次性建立
        self.latest_roll: float = 0.0
        self.latest_pitch: float = 0.0
        self._ang_vel: tuple = (0.0, 0.0, 0.0)
        self._accel: tuple = (0.0, 0.0, 0.0)

        # ---- 就緒旗標 ----
        # warmup 在 __init__ 中同步完成，此旗標只追蹤 IMU 是否已收到第一筆資料。
        # timestep_callback 會在此旗標為 False 時直接 return，不推論、不發布，
        # 讓 servo_client 自然等待直到 adaptation 真正就緒。
        self._imu_ready: bool = False

        # ---- 資料紀錄初始化（方案 C：記憶體緩衝 + 結束時一次性寫檔）----
        # 控制迴圈中只做 numpy 陣列寫入（純記憶體操作，無磁碟 I/O）。
        # 超過 LOG_BUFFER_CAPACITY 筆後不再記錄（不覆寫、不擴充），
        # 避免無限增長，也避免在執行中途因記憶體重新配置而卡頓。
        self.log_dir = "/home/user/ros2_obf_ws/src/sensor_data/"
        os.makedirs(self.log_dir, exist_ok=True)

        self.LOG_BUFFER_CAPACITY = 40000
        # 欄位順序: timestep, roll, pitch,
        #          ang_vel_x/y/z, acc_x/y/z, action_0~5  → 共 15 欄
        self._log_columns = [
            "timestep", "roll", "pitch", "tilt",
            "ang_vel_x", "ang_vel_y", "ang_vel_z",
            "acc_x", "acc_y", "acc_z",
            "action_0", "action_1", "action_2",
            "action_3", "action_4", "action_5",
        ]
        self._log_buf = np.zeros(
            (self.LOG_BUFFER_CAPACITY, len(self._log_columns)), dtype=np.float64
        )
        self._log_count: int = 0  # 已寫入筆數；用於 destroy_node 時知道有效範圍
        self._log_full_warned: bool = False  # 緩衝區滿了只警告一次
        self.get_logger().info(
            f"Data logging buffer initialized "
            f"(capacity={self.LOG_BUFFER_CAPACITY}, dir={self.log_dir})"
        )

        # ---- 模型預熱 ----
        self.get_logger().info("開始進行模型預熱 (Warm-up)...")
        self._warmup_model()
        self.get_logger().info("模型預熱完成！")

        # ---- Publisher / Subscriber ----
        self.publisher = self.create_publisher(
            KneeAction, self.cfg.TOPIC_KNEE_ACTION, self.cfg.QOS_QUEUE_SIZE
        )
        # 就緒信號：warmup 完成且收到第一筆 IMU 後發布一次 True，
        # 讓 servo_client 知道可以開始推進 step。
        
        self._ready_publisher = self.create_publisher(Bool, 'adaptation_ready', adaptation_ready_qos)
        self.create_subscription(
            Imu, self.cfg.TOPIC_IMU, self.imu_callback, self.cfg.QOS_QUEUE_SIZE,
        )
        self.create_subscription(
            StepCPG, self.cfg.TOPIC_TIMESTEP, self.timestep_callback, self.cfg.QOS_QUEUE_SIZE,
        )

        self.get_logger().info("KneeControllerNode ready.")

    # -------------------------------------------------------------------------
    # 工具方法
    # -------------------------------------------------------------------------

    def _warmup_model(self):
        """
        執行一次空推論，讓 TensorRT engine 完成首次 CUDA context 初始化、
        CUDA kernel 載入等開銷，避免第一次收到 /timestep 時產生冷啟動延遲。
        """
        self.trt_runner.infer(self._model_input_buf)
        torch.cuda.current_stream(device=self.device).synchronize()

    # -------------------------------------------------------------------------
    # IMU callback — 純快取，不做任何 GPU 操作
    # -------------------------------------------------------------------------
    def imu_callback(self, msg: Imu):
        # 在 isaac sim 中，imu 座標定義為 x 朝向機器人前方，y 朝向左方
        # 實體機器人是 x 朝右，y 朝前
        q = msg.orientation
        roll, pitch = quat_to_roll_pitch(q.x, q.y, q.z, q.w)
        self.latest_roll = pitch
        self.latest_pitch = -roll

        # 改存 Python tuple，消除高頻路徑上的 GPU tensor 配置
        a = msg.angular_velocity
        self._ang_vel = (a.x, a.y, a.z)

        acc = msg.linear_acceleration
        self._accel = (acc.x, acc.y, acc.z)

        # 第一次收到 IMU 資料時設定旗標，讓 timestep_callback 開始推論，
        # 並發布 /adaptation_ready 通知 servo_client 可以開始推進 step。
        if not self._imu_ready:
            self._imu_ready = True
            self.get_logger().info("First IMU message received.")
            ready_msg = Bool()
            ready_msg.data = True
            self._ready_publisher.publish(ready_msg)

        # 不在此處寫檔；ang_vel / accel 的記錄統一移到 timestep_callback
        # 寫入記憶體 buffer（方案 C），徹底移除高頻路徑上的磁碟 I/O

    # -------------------------------------------------------------------------
    # Timestep callback — 觸發完整推論流程
    # -------------------------------------------------------------------------
    def timestep_callback(self, msg: StepCPG):
        # warmup 與 IMU 都就緒後才開始推論；
        # 未就緒時直接 return，不發布 knee_action，讓 servo_client 自然等待。
        if not self._imu_ready:
            return

        t = int(msg.timestep)
        cpg_val = msg.cpg_value
        #self.get_logger().info(f"t: {t}")
        #self.get_logger().info(f"cpg_val: {cpg_val}")

        # 1. 讀 IMU 快取（Python float/tuple）
        roll = self.latest_roll
        pitch = self.latest_pitch
        # tilt angle according to:
        # tilt = acos(cos(phi_t) * cos(theta_t))
        cos_term = math.cos(roll) * math.cos(pitch)

        # 避免浮點誤差造成 acos domain error
        cos_term = max(-1.0, min(1.0, cos_term))

        tilt = math.acos(cos_term)
        #self.get_logger().info(f"roll: {roll}, pitch: {pitch}")
        #self.get_logger().info(f"ang_vel: {self._ang_vel}")

        # 2. 計算 6 維 IMU 狀態（Python tuple，無 tensor 操作）
        imu_vals = calculate_imu_states(roll, pitch)
        #imu_vals = calculate_imu_states(0, 0)
        # self.get_logger().info(f"imu_vals: {imu_vals}")

        # 3. 填入預配置的 GPU obs buffer（_obs_buf, shape=(16,)）
        #
        #    layout: [0] cpg | [1:7] last_action | [7:13] imu_states | [13:16] ang_vel
        #
        #    - last_action 已在 GPU，直接 GPU→GPU copy
        #    - imu_vals + _ang_vel 寫入預先配置好的 pinned CPU staging buffer
        #      （不重新配置記憶體），再以 non_blocking H2D 搬到 GPU
        self._obs_buf[0] = cpg_val
        self._obs_buf[1:7].copy_(self.last_action)

        ang_vel = self._ang_vel
        self._imu_staging[0] = imu_vals[0]
        self._imu_staging[1] = imu_vals[1]
        self._imu_staging[2] = imu_vals[2]
        self._imu_staging[3] = imu_vals[3]
        self._imu_staging[4] = imu_vals[4]
        self._imu_staging[5] = imu_vals[5]
        self._imu_staging[6] = ang_vel[0]
        self._imu_staging[7] = ang_vel[1]
        self._imu_staging[8] = ang_vel[2]
        self._obs_buf[7:16].copy_(self._imu_staging, non_blocking=True)

        # 4. 寫入環形 buffer（取代 torch.roll，無新 tensor 配置）
        #
        #    _buf_head 指向最舊資料的位置，也是下一次寫入的位置
        #    寫入後遞增，自動循環
        self.obs_history[self._buf_head] = self._obs_buf
        self._buf_head = (self._buf_head + 1) % self.cfg.OBS_BUFFER_LENGTH

        # 5. stride-5 取樣 → (20, 16)
        #
        #    phys_idx 將邏輯偏移 [4, 9, ..., 99] 對應到環形 buffer 的實際位置
        #    邏輯 0 = 最舊，邏輯 99 = 最新
        phys_idx = (
            self._buf_head + self._subsample_offsets
        ) % self.cfg.OBS_BUFFER_LENGTH
        obs_seq = self.obs_history[phys_idx]            # (20, 16), GPU

        # 6. 展平歷史觀測 → 寫入 model input buffer 的前 320 維
        #    （後 8 維 priv_gt 佔位已在 __init__ 固定為 0，不需要每步覆寫）
        self._model_input_buf[0, : self._flat_obs_dim].copy_(obs_seq.reshape(-1))

        # 7. 推論（TensorRT FP16 engine，取代原本的 PyTorch student.act）
        #    輸入固定為 (1, 328)：320 維歷史觀測 + 8 維 priv_gt 佔位（固定 0）。
        #    已驗證：200 組隨機輸入下，換算到實際輸出範圍 0~0.7 後，
        #    最大誤差約 4.65e-3，平均誤差約 6.5e-4，且 GPU 推論時間
        #    （trtexec 量測）平均僅 0.34ms，遠低於控制週期，不構成瓶頸
        raw_action = self.trt_runner.infer(self._model_input_buf)  # (1, 6), GPU, float32
        raw_action = raw_action.squeeze(0)  # (6,)

        # 8. Action 後處理：clamp → 線性映射 [−1, 1] → [0, 0.7]
        action_clamped = torch.clamp(raw_action, -self.cfg.ACTION_CLIP, self.cfg.ACTION_CLIP)
        rl_corrections = (action_clamped + 1.0) / 2.0 * self.cfg.ACTION_SCALE

        # 8b. Warmup 期間（t <= CORRECTION_WARMUP_STEPS）強制修正量為 0。
        #     此時 obs_history 環形 buffer 尚未收滿 OBS_BUFFER_LENGTH 個真實
        #     timestep，20 步歷史序列可能仍含未填滿的初始值，模型輸出不可靠。
        #     強制歸零後再寫入 last_action / 發布，確保歷史紀錄的 action
        #     與實際發布出去、套用到馬達上的 action 完全一致（純 CPG 步態）。
        if t <= self.cfg.CORRECTION_WARMUP_STEPS:
            rl_corrections = torch.zeros_like(rl_corrections)

        # 8c. 輸出濾波（在 warmup 歸零之後套用，讓濾波器狀態在 warmup 期間
        #     保持為 0，warmup 結束後能平滑地從 0 開始爬升）。
        #
        #     濾波器記憶體 = self.last_action（上一步已濾波的輸出），
        #     不需要額外的狀態變數。
        #
        #     重要：濾波後的值同時作為 last_action 回饋進輸入序列，
        #     確保模型「記憶的上一步動作」與馬達實際收到的指令完全一致，
        #     避免模型因看到與現實不符的 last_action 而持續放大輸出。
        _ftype = self.cfg.OUTPUT_FILTER_TYPE
        if _ftype == "ema":
            # 指數移動平均：alpha 越小越平滑，lag 越大
            rl_corrections = (
                self.cfg.OUTPUT_FILTER_ALPHA * rl_corrections
                + (1.0 - self.cfg.OUTPUT_FILTER_ALPHA) * self.last_action
            )
        elif _ftype == "slew":
            # 限制每步最大變化量，抑制突發跳變
            delta = rl_corrections - self.last_action
            delta = torch.clamp(
                delta,
                -self.cfg.OUTPUT_FILTER_MAX_RATE,
                self.cfg.OUTPUT_FILTER_MAX_RATE,
            )
            rl_corrections = self.last_action + delta
        # _ftype == "none"：跳過，行為與修改前完全相同

        # 9. 更新 last_action（GPU tensor，下一步填入 _obs_buf[1:7]）
        #    存的是濾波後的值，確保輸入序列與實際發布的修正量一致。
        self.last_action = rl_corrections

        # 10. 唯一一次 GPU→CPU 同步：取得發布與記錄都共用的數值
        data_list = rl_corrections.detach().cpu().tolist()

        # 11. 發布 /knee_action
        out_msg = KneeAction()
        out_msg.header.stamp = self.get_clock().now().to_msg()
        out_msg.corrections = data_list
        out_msg.timestep = t
        self.publisher.publish(out_msg)

        # 12. 寫入記憶體 log buffer（方案 C：純記憶體操作，無磁碟 I/O）
        #     重複使用第 10 步算好的 data_list，不再對 GPU tensor 呼叫 .item()
        #     超過 LOG_BUFFER_CAPACITY 後不再記錄（不覆寫，直接跳過）
        if self._log_count < self.LOG_BUFFER_CAPACITY:
            ang_vel = self._ang_vel
            accel = self._accel
            row = self._log_buf[self._log_count]
            row[0] = t
            row[1] = roll
            row[2] = pitch
            row[3] = tilt

            row[4], row[5], row[6] = ang_vel[0], ang_vel[1], ang_vel[2]
            row[7], row[8], row[9] = accel[0], accel[1], accel[2]

            row[10], row[11], row[12], row[13], row[14], row[15] = data_list
            self._log_count += 1
        elif not self._log_full_warned:
            self.get_logger().warn(
                f"Log buffer reached capacity ({self.LOG_BUFFER_CAPACITY}), "
                f"further timesteps will not be recorded."
            )
            self._log_full_warned = True

    def destroy_node(self):
        # 節點結束時才一次性把記憶體 buffer 寫出檔案（方案 C）
        try:
            n = self._log_count
            if n > 0:
                data = self._log_buf[:n]

                # =========================================================
                # Tilt statistics
                # tilt = acos(cos(roll) * cos(pitch))
                # =========================================================
                tilt_values = data[:, 3]

                tilt_rms = np.sqrt(np.mean(np.square(tilt_values)))
                tilt_p99 = np.percentile(tilt_values, 99)

                # 儲存 tilt 統計結果
                tilt_stats_path = os.path.join(
                    self.log_dir,
                    "tilt_statistics.txt"
                )

                with open(tilt_stats_path, "w") as f:
                    f.write(f"tilt_rms_rad: {tilt_rms:.8f}\n")
                    f.write(f"tilt_p99_rad: {tilt_p99:.8f}\n")

                for col_idx, col_name in enumerate(self._log_columns):
                    filename = f"{col_name}.txt"
                    path = os.path.join(self.log_dir, filename)
                    # timestep 欄位輸出為整數格式，其餘維持浮點數
                    fmt = "%d" if col_name == "timestep" else "%.8f"
                    np.savetxt(path, data[:, col_idx], fmt=fmt)
                self.get_logger().info(
                    f"Data logging: {n} rows written to {self.log_dir}"
                )
            else:
                self.get_logger().info("Data logging: no rows recorded, nothing to write.")
        except Exception as e:
            self.get_logger().error(f"Failed to write log files: {e}")
        super().destroy_node()


# =============================================================================
# 進入點
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = KneeControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()