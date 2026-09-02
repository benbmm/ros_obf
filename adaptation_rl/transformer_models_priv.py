"""
Cross-Attention Transformer + Privileged State Estimator (單階段端到端訓練)
適用環境: HexapodEnv (Obs: 20 steps x 16 dims 歷史 + 8 維特權狀態 GT = 328 維)

=== 設計概念 ===

cross_out (Transformer Backbone 的歷史編碼輸出) 不再預測「未來狀態」，
改為估計「當前步的特權狀態」(8 維):
    [0:6]  六腳相對地面高度 (feet_clearance，順序 L0,L1,L2,R0,R1,R2)
    [6:8]  機身座標系線速度 v_y, v_z (base_velocity_1, base_velocity_2)
    [8:11] 地形預視相對高度 (terrain preview 30 點中取索引 5,17,29)

=== Actor / Critic 不對稱輸入 ===

Critic 訓練時永遠吃「正確的」特權資訊 (priv_gt)，
Actor 吃的是 priv_gt 與 priv_pred (估計值) 的軟混合:

    priv_use = (1 - alpha) * priv_gt + alpha * priv_pred.detach()

alpha 由 PPOWithPrivEstimator 依訓練進度設定 (0 -> 1)，priv_pred 在混合前
會 detach，因此 policy/value loss 不會透過 priv_use 回傳到
priv_estimator.head / backbone；backbone 與 priv_estimator.head 只由獨立的
estimator loss (MSE(priv_pred, priv_gt)) 更新。

=== Observation 格式 ===

full_states: (N, 328)
    [0:320]   20 steps x 16 dims 歷史觀測 (flatten)
    [320:328] 當前步特權狀態 GT (priv_gt, 8 維, 原始數值)

全部 328 維由外層 RunningStandardScaler 一起正規化，PrivEstimator 不再
自帶 target_mean/target_std。

=== 模組結構 ===

TransformerBackbone (無內部正規化)
├── input_projection: feature_dim -> d_model (Past & Current 共用)
├── pos_embedding: learnable PE for past sequence
├── transformer_encoder: Self-Attention on Past
├── cross_attention_block: current query -> history -> cross_out (with FFN)
└── ln_attn: LayerNorm(d_model)

PrivEstimator(nn.Module)
├── head: d_model -> priv_dim (8)               # priv_pred (估計的8維特權狀態)
└── proj: priv_dim -> priv_feature_dim (32)    # 給 actor/critic 用

Actor / Critic
└── MLP, 輸入維度 = actor_feature_dim + priv_feature_dim (預設 32+32=64)

PPOModel(GaussianMixin, DeterministicMixin, Model)
├── backbone: TransformerBackbone
├── input_projection_actor: feature_dim -> actor_feature_dim (32)
├── priv_estimator: PrivEstimator
├── ln_out: LayerNorm(actor_feature_dim + priv_feature_dim) = LayerNorm(64)
├── actor: Actor (input=64)
├── critic: Critic (input=64)
└── priv_mix_alpha: float，由 PPOWithPrivEstimator 在每次 forward 前設定

=== compute() 資料流 ===

full_states          -> backbone -> cross_out (N, d_model)
full_states[..current..16維..] -> input_projection_actor -> current_actor (N, actor_feature_dim)
full_states[..最後8維..]         -> priv_gt (N, priv_dim)
cross_out            -> priv_estimator.head -> priv_pred (N, priv_dim)

role == "policy":
    priv_use     = (1-alpha)*priv_gt + alpha*priv_pred.detach()
    priv_feature = priv_estimator.proj(priv_use)
    actor_input  = ln_out(cat(current_actor, priv_feature))
    -> actor(actor_input) -> mean, log_std

role == "value":
    priv_feature  = priv_estimator.proj(priv_gt)         # 永遠用GT
    critic_input  = ln_out(cat(current_actor, priv_feature))
    -> critic(critic_input) -> value

兩個 role 都回傳 {"priv_pred": priv_pred, "priv_gt": priv_gt}，
供 agent 計算 estimator loss。
"""

import torch
import torch.nn as nn

from skrl.models.torch import Model, GaussianMixin, DeterministicMixin

# 實機部署只會使用 backbone_type="transformer"，不需要 baseline backbone，
# 為避免部署環境缺少 backbone_baselines_priv.py 而匯入失敗，改為延遲匯入。
try:
    from backbone_baselines_priv import build_alt_backbone
except Exception:
    build_alt_backbone = None


# ============================================================================
# Transformer Encoder Layer (Pre-Norm)
# ============================================================================


class TransformerEncoderLayer(nn.Module):
    """自訂 Transformer Encoder Layer，Pre-Norm + 標準殘差連接。"""

    def __init__(self, d_model, nhead, dim_feedforward, activation="gelu"):
        super().__init__()

        self.self_attn = nn.MultiheadAttention(
            embed_dim=d_model,
            num_heads=nhead,
            batch_first=True,
        )

        self.linear1 = nn.Linear(d_model, dim_feedforward)
        self.linear2 = nn.Linear(dim_feedforward, d_model)

        self.norm1 = nn.LayerNorm(d_model)
        self.norm2 = nn.LayerNorm(d_model)

        if activation == "gelu":
            self.activation = nn.GELU()
        elif activation == "relu":
            self.activation = nn.ReLU()
        else:
            raise ValueError(f"Unsupported activation: {activation}")

    def forward(self, src, src_mask=None, src_key_padding_mask=None):
        """
        Args:
            src: (N, S, d_model)

        Returns:
            out: (N, S, d_model)
        """
        # --- Self-Attention ---
        x = self.norm1(src)
        x, _ = self.self_attn(x, x, x, attn_mask=src_mask, key_padding_mask=src_key_padding_mask, need_weights=False)
        src = src + x

        # --- FFN ---
        x = self.norm2(src)
        x = self.linear2(self.activation(self.linear1(x)))
        src = src + x

        return src


# ============================================================================
# Cross-Attention Block (Pre-Norm 風格)
# ============================================================================


class CrossAttentionBlock(nn.Module):
    """Pre-Norm Cross-Attention Block (with FFN, no residual)."""

    def __init__(self, d_model, nhead, dim_feedforward=256):
        super().__init__()
        self.ln_q = nn.LayerNorm(d_model)
        self.ln_kv = nn.LayerNorm(d_model)

        self.cross_attn = nn.MultiheadAttention(
            embed_dim=d_model,
            num_heads=nhead,
            batch_first=True,
        )

        # FFN (Pre-Norm)
        self.norm_ffn = nn.LayerNorm(d_model)
        self.linear1 = nn.Linear(d_model, dim_feedforward)
        self.linear2 = nn.Linear(dim_feedforward, d_model)
        self.activation = nn.GELU()

    def forward(self, query, key_value):
        # Pre-Norm Cross-Attention
        q = self.ln_q(query)
        kv = self.ln_kv(key_value)
        cross_out, _ = self.cross_attn(q, kv, kv, need_weights=False)

        # FFN (Pre-Norm, no residual)
        ffn_in = self.norm_ffn(cross_out)
        cross_out = self.linear2(self.activation(self.linear1(ffn_in)))

        return cross_out


# ============================================================================
# TransformerBackbone: Encoder + Cross-Attention (無內部正規化)
# ============================================================================


class TransformerBackbone(nn.Module):
    """
    共享的 Transformer 編碼器。

    輸入: (N, flat_dim)，flat_dim >= total_seq_len * feature_dim
          (多出來的維度，例如 priv_gt，會被忽略)
    輸出:
        cross_out: (N, d_model) — 經過 CrossAttention + FFN + ln_attn

    不做內部 z-score 正規化：輸入的整個 observation 已由外層
    RunningStandardScaler 一起正規化。
    """

    def __init__(
        self,
        d_model=64,
        nhead=4,
        num_encoder_layers=2,
        dim_feedforward=128,
        total_seq_len=20,
        feature_dim=16,
        num_query_tokens=4,  
    ):
        super().__init__()
        self.d_model = d_model
        self.total_seq_len = total_seq_len
        self.feature_dim = feature_dim
        self.past_seq_len = total_seq_len - 1
        self.flat_obs_dim = total_seq_len * feature_dim
        self.num_query_tokens = num_query_tokens   # ← 新增
        self.backbone_output_dim = num_query_tokens * d_model  # ← 新增

        # Input Projection: feature_dim -> d_model (Past 和 Current query 共用)
        self.input_projection = nn.Sequential(
            nn.Linear(feature_dim, d_model),
            nn.LayerNorm(d_model),
            nn.ReLU(),
        )
        self.current_query_projection = nn.Sequential(
            nn.Linear(feature_dim, num_query_tokens * d_model),
            nn.ReLU(),
        )

        # Learnable Positional Encoding (僅用於 Past 序列)
        self.pos_embedding = nn.Parameter(
            torch.zeros(1, self.past_seq_len, d_model)
        )
        nn.init.trunc_normal_(self.pos_embedding, std=0.02)

        # Transformer Encoder (Self-Attention on Past)
        self.transformer_encoder = nn.ModuleList([
            TransformerEncoderLayer(
                d_model=d_model,
                nhead=nhead,
                dim_feedforward=dim_feedforward,
            )
            for _ in range(num_encoder_layers)
        ])

        # Cross-Attention Block
        self.cross_attention_block = CrossAttentionBlock(
            d_model=d_model,
            nhead=nhead,
            dim_feedforward=dim_feedforward,
        )

        self.ln_attn = nn.LayerNorm(self.backbone_output_dim)

    def forward(self, full_states: torch.Tensor):
        """
        Args:
            full_states: (N, flat_dim)，flat_dim >= flat_obs_dim

        Returns:
            cross_out: (N, d_model)
        """
        batch_size = full_states.shape[0]

        obs = full_states[:, : self.flat_obs_dim]
        x = obs.view(batch_size, self.total_seq_len, self.feature_dim)

        # 拆分 Past 和 Current
        past = x[:, :-1, :]      # (N, past_seq_len, feature_dim)
        current = x[:, -1:, :]   # (N, 1, feature_dim)

        # Input Projection
        past_proj = self.input_projection(past)        # (N, past_seq_len, d_model)
        current_obs = x[:, -1, :]   # (N, feature_dim)，不再用 input_projection
        query = self.current_query_projection(current_obs)          # (N, num_query_tokens * d_model)
        query = query.view(query.shape[0], self.num_query_tokens, self.d_model)  # (N, num_query_tokens, d_model)

        # Positional Encoding (僅 Past)
        past_proj = past_proj + self.pos_embedding

        # Transformer Encoder (逐層呼叫)
        encoder_out = past_proj
        for layer in self.transformer_encoder:
            encoder_out = layer(encoder_out)

        # Cross-Attention
        cross_out = self.cross_attention_block(query=query, key_value=encoder_out)  # (N, num_query_tokens, d_model)
        cross_out = self.ln_attn(cross_out.reshape(cross_out.shape[0], -1))        # (N, backbone_output_dim)

        return cross_out


# ============================================================================
# PrivEstimator: 估計特權狀態 (feet_clearance x6 + base_velocity_y/z)
# ============================================================================


class PrivEstimator(nn.Module):
    """
    從 cross_out 估計 8 維特權狀態，並提供投影層供 actor/critic 使用。

    head: cross_out (d_model) -> priv_pred (priv_dim)
        priv_pred 與 priv_gt 處在同一個正規化空間 (由外層
        RunningStandardScaler 正規化)，兩者直接做 MSE 即可，不需要
        額外的 target_mean/target_std。

    proj: priv_vec (priv_dim) -> priv_feature (priv_feature_dim)
        actor 用 priv_use (GT/估計值軟混合) 投影；
        critic 用 priv_gt 投影。共用同一組權重。
    """

    def __init__(self, backbone_output_dim=256, priv_dim=8, priv_feature_dim=32):
        
        super().__init__()
        self.head = nn.Linear(backbone_output_dim, priv_dim)
        self.priv_dim = priv_dim

        self.proj = nn.Sequential(
            nn.Linear(priv_dim, priv_feature_dim),
            nn.LayerNorm(priv_feature_dim),
            nn.ReLU(),
        )

    def estimate(self, cross_out: torch.Tensor) -> torch.Tensor:
        """cross_out (N, d_model) -> priv_pred (N, priv_dim)"""
        return self.head(cross_out)

    def project(self, priv_vec: torch.Tensor) -> torch.Tensor:
        """priv_vec (N, priv_dim) -> priv_feature (N, priv_feature_dim)"""
        return self.proj(priv_vec)


# ============================================================================
# Actor: 策略網路
# ============================================================================


class Actor(nn.Module):
    """
    Actor MLP Head.

    輸入: actor_input (N, input_dim) — ln_out(cat(current_actor, priv_feature))
    輸出: action mean (N, action_dim)
    """

    def __init__(self, input_dim=64, hidden_dim=64, action_dim=6):
        super().__init__()
        self.mlp = nn.Sequential(
            nn.Linear(input_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
        )
        self.mean_layer = nn.Linear(hidden_dim, action_dim)
        self.log_std_parameter = nn.Parameter(torch.zeros(action_dim))

    def forward(self, actor_input: torch.Tensor):
        """
        Returns:
            mean:          (N, action_dim)
            log_std_param: (action_dim,)
        """
        x = self.mlp(actor_input)
        mean = self.mean_layer(x)
        return mean, self.log_std_parameter


# ============================================================================
# Critic: 價值網路
# ============================================================================


class Critic(nn.Module):
    """
    Critic MLP Head.

    輸入: critic_input (N, input_dim) — ln_out(cat(current_actor, priv_feature(priv_gt)))
    輸出: state value (N, 1)
    """

    def __init__(self, input_dim=64, hidden_dim=64):
        super().__init__()
        self.mlp = nn.Sequential(
            nn.Linear(input_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
        )
        self.value_layer = nn.Linear(hidden_dim, 1)

    def forward(self, critic_input: torch.Tensor):
        """
        Returns:
            value: (N, 1)
        """
        x = self.mlp(critic_input)
        value = self.value_layer(x)
        return value


# ============================================================================
# PPOModel: 組合所有模組，供 skrl PPO 使用
# ============================================================================


class PPOModel(GaussianMixin, DeterministicMixin, Model):
    """
    完整的 PPO 模型，組合 TransformerBackbone + PrivEstimator + Actor + Critic。

    Critic 永遠使用正確的特權資訊 (priv_gt)；
    Actor 使用 priv_gt 與 priv_pred 的軟混合 (priv_mix_alpha 控制)。
    """

    def __init__(
        self,
        observation_space,
        action_space,
        device,
        # --- PPO Gaussian Policy ---
        clip_actions=False,
        clip_log_std=True,
        min_log_std=-20,
        max_log_std=2,
        reduction="sum",
        # --- Backbone 設定 ---
        backbone_type="transformer",
        backbone_arch_cfg=None,
        d_model=64,
        nhead=4,
        num_encoder_layers=2,
        dim_feedforward=128,
        total_seq_len=20,
        feature_dim=16,
        # --- Privileged State Estimator 設定 ---
        priv_dim=8,
        priv_feature_dim=32,
        actor_feature_dim=32,
        hidden_dim=64,
        num_query_tokens=4,
        seed: int = 42,  
    ):
        Model.__init__(
            self,
            observation_space=observation_space,
            action_space=action_space,
            device=device,
        )
        GaussianMixin.__init__(
            self,
            clip_actions=clip_actions,
            clip_log_std=clip_log_std,
            min_log_std=min_log_std,
            max_log_std=max_log_std,
            reduction=reduction,
            role="policy",
        )
        DeterministicMixin.__init__(self, clip_actions=False, role="value")

        action_dim = self.num_actions

        self.feature_dim = feature_dim
        self.priv_dim = priv_dim
        self.backbone_type = str(backbone_type).lower()

        # --- Backbone ---
        if self.backbone_type == "transformer":
            self.backbone = TransformerBackbone(
                d_model=d_model,
                nhead=nhead,
                num_encoder_layers=num_encoder_layers,
                dim_feedforward=dim_feedforward,
                total_seq_len=total_seq_len,
                feature_dim=feature_dim,
                num_query_tokens=num_query_tokens,
            )
        else:
            # MLP / CNN baseline backbone (見 backbone_baselines_priv.py)
            # 介面與 TransformerBackbone 一致: forward(full_states) -> cross_out
            if build_alt_backbone is None:
                raise ImportError(
                    "backbone_type 非 'transformer' 時需要 backbone_baselines_priv.py，"
                    "但目前環境中找不到這個模組（部署環境預設未包含，因為只用得到 transformer backbone）。"
                )
            self.backbone = build_alt_backbone(
                backbone_type=self.backbone_type,
                total_seq_len=total_seq_len,
                feature_dim=feature_dim,
                arch_cfg=backbone_arch_cfg,
            )
        self.flat_obs_dim = self.backbone.flat_obs_dim  # total_seq_len * feature_dim

        torch.manual_seed(seed)

        # --- Actor/Critic 共用的「當前狀態」投影 ---
        self.input_projection_actor = nn.Sequential(
            nn.Linear(feature_dim, actor_feature_dim),
            nn.LayerNorm(actor_feature_dim),
            nn.ReLU(),
        )

        # --- Privileged State Estimator ---
        self.priv_estimator = PrivEstimator(
            backbone_output_dim=self.backbone.backbone_output_dim,  # ← 原本是 d_model=d_model
            priv_dim=priv_dim,
            priv_feature_dim=priv_feature_dim,
        )

        ln_out_dim = actor_feature_dim + priv_feature_dim
        self.ln_out = nn.LayerNorm(ln_out_dim)

        self.actor = Actor(input_dim=ln_out_dim, hidden_dim=hidden_dim, action_dim=action_dim)
        self.critic = Critic(input_dim=ln_out_dim, hidden_dim=hidden_dim)

        # 軟混合係數：0 = actor 全用 priv_gt，1 = actor 全用 priv_pred。
        # 由 PPOWithPrivEstimator 在每次 forward 前依訓練進度設定。
        self.priv_mix_alpha = 0.0

        # --- 印出參數量 ---
        total_params = sum(p.numel() for p in self.parameters())
        trainable_params = sum(p.numel() for p in self.parameters() if p.requires_grad)
        print(f"[PPOModel] backbone_type={self.backbone_type}")
        if self.backbone_type == "transformer":
            print(f"[PPOModel] d_model={d_model}, nhead={nhead}, encoder_layers={num_encoder_layers}")
        print(f"[PPOModel] backbone_output_dim={self.backbone.backbone_output_dim}, "
              f"backbone params={sum(p.numel() for p in self.backbone.parameters()):,}")
        print(f"[PPOModel] Sequence: {total_seq_len} steps x {feature_dim} features "
              f"(flat_obs_dim={self.flat_obs_dim})")
        print(f"[PPOModel] Priv dim={priv_dim}, priv_feature_dim={priv_feature_dim}, "
              f"actor_feature_dim={actor_feature_dim}")
        print(f"[PPOModel] Actor/Critic input dim: {ln_out_dim}")
        print(f"[PPOModel] Total parameters: {total_params:,}")
        print(f"[PPOModel] Trainable parameters: {trainable_params:,}")

    # =========================================================================
    # Role 分流
    # =========================================================================

    def act(self, inputs, role):
        if role == "policy":
            return GaussianMixin.act(self, inputs, role)
        elif role == "value":
            return DeterministicMixin.act(self, inputs, role)

    def compute(self, inputs, role):
        """
        full_states: (N, flat_obs_dim + priv_dim)
            [:flat_obs_dim]                     -> 20 steps x 16 dims 歷史觀測
            [flat_obs_dim:flat_obs_dim+priv_dim] -> priv_gt (8維特權狀態 GT)
        """
        full_states = inputs["states"]

        cross_out = self.backbone(full_states)  # (N, d_model)

        # 當前步原始觀測 (歷史序列中最後一步，注意不能用 full_states[:, -feature_dim:]，
        # 因為 full_states 後面還接著 priv_gt)
        current = full_states[:, self.flat_obs_dim - self.feature_dim: self.flat_obs_dim]
        current_actor = self.input_projection_actor(current)  # (N, actor_feature_dim)

        # 特權狀態 GT (已由外層 RunningStandardScaler 正規化)
        priv_gt = full_states[:, self.flat_obs_dim: self.flat_obs_dim + self.priv_dim]

        # 特權狀態估計值 (與 priv_gt 同一正規化空間)
        priv_pred = self.priv_estimator.estimate(cross_out)  # (N, priv_dim)

        if role == "policy":
            alpha = self.priv_mix_alpha
            priv_use = (1.0 - alpha) * priv_gt + alpha * priv_pred.detach()
            priv_feature = self.priv_estimator.project(priv_use)

            actor_input = self.ln_out(torch.cat((current_actor, priv_feature), dim=-1))
            mean, log_std = self.actor(actor_input)
            return mean, log_std, {"priv_pred": priv_pred, "priv_gt": priv_gt}

        elif role == "value":
            # Critic 永遠使用正確的特權資訊
            priv_feature = self.priv_estimator.project(priv_gt)

            critic_input = self.ln_out(torch.cat((current_actor, priv_feature), dim=-1))
            value = self.critic(critic_input)
            return value, {"priv_pred": priv_pred, "priv_gt": priv_gt}