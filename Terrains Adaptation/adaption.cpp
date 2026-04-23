"""
膝關節控制器 ROS2 節點（主節點）

功能:
    訂閱 /imu_data 與 /timestep，每收到一次 /timestep 即執行一次推論，
    將 6 維膝關節修正量（rl_corrections, 範圍 0~0.7）發布至 /knee_action。

本檔案整合以下邏輯:
    1. CPG 資料讀取（純浮點數一行一數）
    2. 四元數 → roll/pitch 轉換 + 6 維 IMU 狀態展開
    3. skrl PPOWithPredictor agent 載入
    4. ROS2 節點主體（兩個 callback + 推論主迴圈）
"""

import math
from typing import Tuple
from scipy.spatial.transform import Rotation

import numpy as np
import torch
from gymnasium.spaces import Box

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from interfaces.msg import KneeAction

# skrl
from skrl.agents.torch.ppo import PPO_DEFAULT_CONFIG
from skrl.memories.torch import RandomMemory
from skrl.resources.preprocessors.torch import RunningStandardScaler
from skrl.resources.schedulers.torch import KLAdaptiveRL

# 自訂模型與 Agent（使用者需保證這兩個檔案在 PYTHONPATH 可存取）
from transformer_models import PPOModel
from custom_ppo_agent import PPOWithPredictor

# 本專案
from node_config import NodeConfig


# =============================================================================
# 1. CPG 檔案載入
# =============================================================================

def load_cpg_l0_hip(file_path: str, device: torch.device) -> torch.Tensor:
    """
    讀取純浮點數檔案，每行一個 float。

    Args:
        file_path: YYout11.txt 的絕對路徑（L0 腿髖關節訊號）
        device: 資料要放置的 torch device

    Returns:
        cpg_tensor: (max_steps,) 一維 tensor
    """
    values = []
    with open(file_path, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            values.append(float(line))

    if len(values) == 0:
        raise RuntimeError(f"CPG 檔案為空: {file_path}")

    cpg_tensor = torch.tensor(values, dtype=torch.float32, device=device)
    return cpg_tensor


# =============================================================================
# 2. IMU 工具
# =============================================================================

def calculate_imu_states(roll: float, pitch: float, device: torch.device) -> torch.Tensor:
    """
    將 roll/pitch 展開為 6 維 IMU 狀態（與訓練環境 _calculate_imu_states 完全一致）。

    Returns:
        imu_states: (6,) tensor on device
    """
    sqrt_half = math.sqrt(0.5)
    imu_states = [
        (-pitch - roll) * sqrt_half,   # [0]
        -roll,                         # [1]
        (pitch - roll) * sqrt_half,    # [2]
        (pitch + roll) * sqrt_half,    # [3]
        roll,                          # [4]
        (-pitch + roll) * sqrt_half,   # [5]
    ]
    return torch.tensor(imu_states, dtype=torch.float32, device=device)


# =============================================================================
# 3. skrl Agent 載入
# =============================================================================

def load_agent(cfg: NodeConfig, device: torch.device) -> PPOWithPredictor:
    """
    建立 PPOModel + PPOWithPredictor agent，載入 checkpoint，設為 eval 模式。

    流程與 play_transformer.py 一致：
        1. 以 NodeConfig 超參數建立 PPOModel
        2. 建 dummy RandomMemory（推理不會用到，但 agent.init 需要）
        3. 組裝 PPO config dict（含 RunningStandardScaler 設定）
        4. 建 PPOWithPredictor
        5. agent.load(checkpoint)  ← 一併載入 scaler 統計量
        6. agent.set_running_mode("eval")
    """
    flat_obs_dim = cfg.TOTAL_SEQ_LEN * cfg.FEATURE_DIM  # 20 * 16 = 320

    # --- Dummy gym spaces（沒有真實 env，手動造）---
    observation_space = Box(
        low=-np.inf, high=np.inf, shape=(flat_obs_dim,), dtype=np.float32
    )
    action_space = Box(
        low=-1.0, high=1.0, shape=(cfg.ACTION_DIM,), dtype=np.float32
    )

    # --- 建 PPOModel ---
    # normalize_input=False 因為 Stage 2 由外層 RunningStandardScaler 負責
    model = PPOModel(
        observation_space=observation_space,
        action_space=action_space,
        device=device,
        clip_actions=cfg.CLIP_ACTIONS,
        clip_log_std=cfg.CLIP_LOG_STD,
        min_log_std=cfg.MIN_LOG_STD,
        max_log_std=cfg.MAX_LOG_STD,
        reduction=cfg.REDUCTION,
        d_model=cfg.D_MODEL,
        nhead=cfg.NHEAD,
        num_encoder_layers=cfg.NUM_ENCODER_LAYERS,
        dim_feedforward=cfg.DIM_FEEDFORWARD,
        total_seq_len=cfg.TOTAL_SEQ_LEN,
        feature_dim=cfg.FEATURE_DIM,
        predictor_output_dim=cfg.PREDICTOR_OUTPUT_DIM,
        normalize_input=False,
    )

    # Policy 與 Value 共用同一個 PPOModel 實例
    models = {"policy": model, "value": model}

    # --- Dummy memory（推理用不到）---
    memory = RandomMemory(memory_size=16, num_envs=1, device=device)

    # --- PPO config（只設載入 checkpoint 必要的欄位）---
    ppo_cfg = PPO_DEFAULT_CONFIG.copy()
    ppo_cfg["state_preprocessor"] = RunningStandardScaler
    ppo_cfg["value_preprocessor"] = RunningStandardScaler
    ppo_cfg["state_preprocessor_kwargs"] = {"size": observation_space, "device": device}
    ppo_cfg["value_preprocessor_kwargs"] = {"size": 1, "device": device}
    ppo_cfg["learning_rate_scheduler"] = KLAdaptiveRL
    ppo_cfg["learning_rate_scheduler_kwargs"] = {}
    ppo_cfg["experiment"]["write_interval"] = 0
    ppo_cfg["experiment"]["checkpoint_interval"] = 0

    # --- 建 PPOWithPredictor ---
    # predictor_loss_weight / backbone_freeze_timesteps 對 eval 不影響，給預設值即可
    agent = PPOWithPredictor(
        models=models,
        memory=memory,
        cfg=ppo_cfg,
        observation_space=observation_space,
        action_space=action_space,
        device=device,
        predictor_loss_weight=0.001,
        predictor_target_dim=cfg.PREDICTOR_OUTPUT_DIM,
        total_seq_len=cfg.TOTAL_SEQ_LEN,
        feature_dim=cfg.FEATURE_DIM,
        backbone_freeze_timesteps=0,
    )

    # --- 載入 checkpoint ---
    agent.load(cfg.CHECKPOINT_PATH)

    # --- 設為 eval 模式 ---
    agent.set_running_mode("eval")

    return agent


# =============================================================================
# 4. ROS2 節點主體
# =============================================================================

class KneeControllerNode(Node):
    """
    單執行緒 ROS2 節點。
    - /imu_data  callback: 高頻，僅快取 roll/pitch/angular_velocity，不做推論。
    - /timestep  callback: 觸發完整推論流程，發布 /knee_action。
    """

    def __init__(self):
        super().__init__("knee_controller")

        self.cfg = NodeConfig()
        self.device = torch.device(self.cfg.DEVICE)

        self.get_logger().info(f"Using device: {self.device}")

        # ---- 1. 載入 CPG ----
        self.get_logger().info(f"Loading CPG from: {self.cfg.CPG_FILE_PATH}")
        self.cpg_l0_hip = load_cpg_l0_hip(self.cfg.CPG_FILE_PATH, self.device)
        self.cpg_max_steps = self.cpg_l0_hip.shape[0]
        self.get_logger().info(f"CPG loaded: {self.cpg_max_steps} steps")

        # ---- 2. 載入 Agent ----
        self.get_logger().info(f"Loading checkpoint: {self.cfg.CHECKPOINT_PATH}")
        self.agent = load_agent(self.cfg, self.device)
        self.get_logger().info("Agent loaded and set to eval mode")

        # ---- 3. 初始化內部狀態 ----
        # 環形 obs buffer: (100, 16)
        self.obs_history = torch.zeros(
            self.cfg.OBS_BUFFER_LENGTH, self.cfg.FEATURE_DIM, device=self.device
        )

        # last_action: 上一步 rl_corrections（已後處理，範圍 0~0.7）
        self.last_action = torch.zeros(self.cfg.ACTION_DIM, device=self.device)

        # IMU 快取
        self.latest_roll: float = 0.0
        self.latest_pitch: float = 0.0
        self.latest_ang_vel: torch.Tensor = torch.zeros(3, device=self.device)

        # 預先計算 stride-5 取樣的 indices: [4, 9, 14, ..., 99]
        self._subsample_indices = torch.tensor(
            [
                self.cfg.OBS_SUBSAMPLE_STRIDE * (i + 1) - 1
                for i in range(self.cfg.OBS_OUTPUT_SEQ_LEN)
            ],
            dtype=torch.long,
            device=self.device,
        )  # shape (20,)

        # ---- 4. Publisher / Subscriber ----
        self.publisher = self.create_publisher(
            KneeAction, self.cfg.TOPIC_KNEE_ACTION, self.cfg.QOS_QUEUE_SIZE
        )
        self.create_subscription(
            Imu,
            self.cfg.TOPIC_IMU,
            self.imu_callback,
            self.cfg.QOS_QUEUE_SIZE,
        )
        self.create_subscription(
            Int64,
            self.cfg.TOPIC_TIMESTEP,
            self.timestep_callback,
            self.cfg.QOS_QUEUE_SIZE,
        )

        self.get_logger().info("KneeControllerNode ready.")

    # -------------------------------------------------------------------------
    # IMU callback — 純快取，不觸發推論
    # -------------------------------------------------------------------------
    def imu_callback(self, msg: Imu):
        q = msg.orientation
        roll, pitch, _ = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz', degrees=False)
        self.latest_roll = roll
        self.latest_pitch = pitch

        a = msg.angular_velocity
        # 重用同一個 tensor，避免頻繁配置（可選：直接 new tensor 也可以）
        self.latest_ang_vel = torch.tensor(
            [a.x, a.y, a.z], dtype=torch.float32, device=self.device
        )

    # -------------------------------------------------------------------------
    # Timestep callback — 觸發完整推論流程
    # -------------------------------------------------------------------------
    def timestep_callback(self, msg: Int64):
        t = int(msg.data)

        # 1. CPG scalar（L0 hip）
        cpg_single = self.cpg_l0_hip[t % self.cpg_max_steps].unsqueeze(0)  # (1,)

        # 2. 讀 IMU 快取
        roll = self.latest_roll
        pitch = self.latest_pitch
        ang_vel = self.latest_ang_vel  # (3,)

        # 3. 6 維 IMU 狀態
        imu_states = calculate_imu_states(roll, pitch, self.device)  # (6,)

        # 4. 組裝 current_obs (16,)
        current_obs = torch.cat(
            [
                cpg_single,        # [0]     (1,)
                self.last_action,  # [1:7]   (6,)
                imu_states,        # [7:13]  (6,)
                ang_vel,           # [13:16] (3,)
            ]
        )

        # 5. 更新環形 buffer
        self.obs_history = torch.roll(self.obs_history, shifts=-1, dims=0)
        self.obs_history[-1] = current_obs

        # 6. stride-5 取樣 → (20, 16)
        obs_seq = self.obs_history[self._subsample_indices]

        # 7. 展平 → (1, 320) 並做推論
        flat_obs_dim = self.cfg.TOTAL_SEQ_LEN * self.cfg.FEATURE_DIM
        obs_flat = obs_seq.reshape(1, flat_obs_dim)

        with torch.inference_mode():
            # agent.act 內部會套用 RunningStandardScaler
            outputs = self.agent.act(obs_flat, timestep=0, timesteps=0)
            raw_action = outputs[-1].get("mean_actions", outputs[0])  # (1, 6)
        raw_action = raw_action.squeeze(0)  # (6,)

        # 8. Action 後處理：clamp → 線性映射 → 縮放
        action_clamped = torch.clamp(raw_action, -self.cfg.ACTION_CLIP, self.cfg.ACTION_CLIP)
        rl_corrections = (action_clamped + 1.0) / 2.0 * self.cfg.ACTION_SCALE

        # 9. 更新 last_action（存後處理後的值）
        self.last_action = rl_corrections

        # 10. 發布 /knee_action
        data_list = rl_corrections.detach().cpu().tolist()
        out_msg.header.stamp = self.get_clock().now().to_msg()
        out_msg.corrections = data_list   # float32[6]
        out_msg.timestep = t              # 對應的 timestep
        self.publisher.publish(out_msg)


# =============================================================================
# 5. 進入點
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = KneeControllerNode()
    try:
        rclpy.spin(node)  # SingleThreadedExecutor 為預設
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()