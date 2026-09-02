"""
節點參數設定 — 唯一的參數入口。
所有路徑、超參數、訓練 config 對應的數值，只能在這裡修改。

對應訓練設定: skrl_ppo_cfg_priv.yaml
模型類別: PPOModel (transformer_models_priv.py)

=== 與舊版 (HexaPrivStudentModel) 的差異 ===
    - 模型輸入不再只是 320 維歷史觀測，而是 328 維
      = SEQ_LEN*FEATURE_DIM (320, 歷史觀測) + PRIV_DIM (8, 特權狀態 priv_gt 佔位)
    - 輸入正規化不再內建於模型 (obs_mean/obs_std)，
      而是由訓練時外層的 skrl RunningStandardScaler 對整個 328 維一起做正規化，
      統計值存在 checkpoint 的 "state_preprocessor" key 裡 (不在 policy state_dict 中)。
      實機推論時必須在 export_onnx.py 把這段正規化邏輯一起烘焙進 ONNX/TensorRT。
    - 新增 PRIV_MIX_ALPHA：實機沒有 feet_clearance / base_velocity 等特權感測器，
      因此固定使用 alpha=1.0，代表 actor 完全依賴 PrivEstimator 估計出來的 priv_pred，
      priv_gt 部分純粹補維度用，直接填 0 即可 (數值不影響 actor 輸出)。
"""


class NodeConfig:
    # =========================================================================
    # 路徑（實際部署時請修改）
    # =========================================================================
    CHECKPOINT_PATH = "/home/user/ros2_obf_ws/src/adaptation_rl/adaptation_rl/07010141_best_priv_real_best_max_delta_0.1.pt"

    # TensorRT engine 路徑（由 export_onnx.py + convert_trt.sh 產生，
    # adaptation_rl.py 推論時直接從這裡讀取，需與 convert_trt.sh 的
    # ENGINE_PATH 輸出路徑一致）
    ENGINE_PATH = "/home/user/ros2_obf_ws/src/adaptation_rl/adaptation_rl/07010141_best_priv_real_best_max_delta_0.1.engine"

    # =========================================================================
    # 硬體
    # =========================================================================
    DEVICE = "cuda"   # "cuda" 或 "cpu"

    # =========================================================================
    # Action 後處理（與訓練環境 _pre_physics_step 一致）
    # 最終輸出範圍 = 0.0 ~ 0.7
    # =========================================================================
    ACTION_CLIP = 1.0    # clamp(raw_action, -1.0, +1.0)
    ACTION_SCALE = 0.0   # (clamped + 1) / 2 * 0.7

    # =========================================================================
    # 模型超參數（必須與 checkpoint 訓練時一致）
    # 對應 skrl_ppo_cfg_priv.yaml -> models.custom
    # =========================================================================

    # --- 觀察序列（歷史部分）---
    SEQ_LEN = 20              # 送入 backbone 的歷史序列長度（obs_output_seq_len）
    FEATURE_DIM = 16          # 每步觀察維度（visible_dim）

    # --- Transformer Backbone ---
    D_MODEL = 64
    NHEAD = 2
    NUM_ENCODER_LAYERS = 2     # 對應 yaml num_encoder_layers
    DIM_FEEDFORWARD = 128
    NUM_QUERY_TOKENS = 4       # 對應 yaml num_query_tokens

    # --- Privileged State Estimator 設定 ---
    PRIV_DIM = 8               # 特權狀態維度 (feet_clearance x6 + base_velocity_y/z)
    PRIV_FEATURE_DIM = 32      # priv -> actor/critic 的投影維度
    ACTOR_FEATURE_DIM = 32     # 當前觀測 -> actor/critic 的投影維度 (32+32=64)

    # --- Actor / Critic Head（新模型 actor/critic 共用同一個 hidden_dim）---
    HIDDEN_DIM = 128

    # --- 實機推論固定參數 ---
    # 真實機器人沒有特權感測器，alpha=1.0 代表 actor 完全使用 PrivEstimator
    # 估計出來的 priv_pred，priv_gt 完全不影響 actor 輸出。
    PRIV_MIX_ALPHA = 1.0

    # --- 輸出濾波（修正量平滑，緩解 sim-to-real overshoot）---
    #
    # OUTPUT_FILTER_TYPE:
    #   "ema"   — 指數移動平均。適合持續高頻振盪。
    #             y[t] = alpha * raw[t] + (1 - alpha) * y[t-1]
    #             alpha 越小越平滑，lag 越大；建議從 0.3~0.5 開始調。
    #   "slew"  — 限制每步最大變化量。適合突發跳變（step change）。
    #             delta = clamp(raw - last, -max_rate, +max_rate)
    #             max_rate 建議為最大修正量 0.7 的 10%~20%（即 0.07~0.14）。
    #   "none"  — 關閉濾波（與修改前行為完全相同）。
    #
    # 注意：濾波後的值同時作為 last_action 回饋進輸入序列，
    #       確保模型「記憶」與馬達實際收到的指令一致。
    OUTPUT_FILTER_TYPE  = "ema"   # "ema" | "slew" | "none"
    OUTPUT_FILTER_ALPHA    = 0.2  # 僅 EMA 使用
    OUTPUT_FILTER_MAX_RATE = 0.07 # 僅 Slew 使用（單步最大變化量，單位與修正量相同）

    # --- 輸入正規化 (對應 skrl RunningStandardScaler 預設參數，train_priv.py 未覆寫) ---
    PREPROCESSOR_EPSILON = 1e-8
    PREPROCESSOR_CLIP_THRESHOLD = 5.0

    # --- PPO Gaussian Policy 參數 ---
    ACTION_DIM = 6
    CLIP_ACTIONS = False
    CLIP_LOG_STD = True
    MIN_LOG_STD = -20.0
    MAX_LOG_STD = 2.0
    REDUCTION = "sum"

    # =========================================================================
    # 模型總輸入維度（送進 backbone + RunningStandardScaler 的完整向量長度）
    #   = SEQ_LEN * FEATURE_DIM (320, 歷史觀測) + PRIV_DIM (8, priv_gt 佔位，固定填 0)
    # =========================================================================
    TOTAL_INPUT_DIM = SEQ_LEN * FEATURE_DIM + PRIV_DIM  # 320 + 8 = 328

    # =========================================================================
    # 觀察序列管理（與訓練環境一致）
    # =========================================================================
    OBS_BUFFER_LENGTH = 100      # 環形 buffer 長度
    OBS_SUBSAMPLE_STRIDE = 5     # stride-5 取樣
    OBS_OUTPUT_SEQ_LEN = 20      # 送入模型的序列長度（= SEQ_LEN）

    # 前 N 步（timestep）強制修正量為 0：
    # OBS_BUFFER_LENGTH=100 代表環形 buffer 要收滿 100 個真實 timestep
    # 才能組出完全由真實資料填滿、不含未初始化內容的 20 步歷史序列。
    # 在這之前模型輸出的修正量不可靠，因此固定發布 0（純 CPG 步態），
    # 同時 last_action 歷史也記錄 0，確保「歷史紀錄的 action」與
    # 「實際發布/套用到馬達上的 action」保持一致。
    CORRECTION_WARMUP_STEPS = 100

    # =========================================================================
    # ROS2 Topic 名稱
    # =========================================================================
    TOPIC_IMU = "/imu/data"
    TOPIC_TIMESTEP = "/timestep"
    TOPIC_KNEE_ACTION = "/knee_action"

    # Subscriber / Publisher QoS queue size
    QOS_QUEUE_SIZE = 1