from __future__ import annotations

from typing import Any

import torch
import torch.nn as nn

from skrl.models.torch import DeterministicMixin, GaussianMixin, Model

try:
    from scripts.skrl.hexa_privilege_mlp.hexa_priv_distill_utils import activation_layer
except Exception:
    from adaptation_rl.hexa_priv_distill_utils import activation_layer


class TransformerEncoderLayer(nn.Module):
    def __init__(self, d_model: int, nhead: int, dim_feedforward: int, activation: str = "gelu"):
        super().__init__()
        self.self_attn = nn.MultiheadAttention(d_model, nhead, batch_first=True)
        self.norm1 = nn.LayerNorm(d_model)
        self.norm2 = nn.LayerNorm(d_model)
        self.linear1 = nn.Linear(d_model, dim_feedforward)
        self.linear2 = nn.Linear(dim_feedforward, d_model)
        self.activation = activation_layer(activation)()

    def forward(self, src: torch.Tensor) -> torch.Tensor:
        x = self.norm1(src)
        x, _ = self.self_attn(x, x, x, need_weights=False)
        src = src + x
        x = self.norm2(src)
        return src + self.linear2(self.activation(self.linear1(x)))


class CrossAttentionBlock(nn.Module):
    def __init__(self, d_model: int, nhead: int, dim_feedforward: int, activation: str = "gelu"):
        super().__init__()
        self.ln_q = nn.LayerNorm(d_model)
        self.ln_kv = nn.LayerNorm(d_model)
        self.cross_attn = nn.MultiheadAttention(d_model, nhead, batch_first=True)
        self.norm_ffn = nn.LayerNorm(d_model)
        self.linear1 = nn.Linear(d_model, dim_feedforward)
        self.linear2 = nn.Linear(dim_feedforward, d_model)
        self.activation = activation_layer(activation)()

    def forward(self, query: torch.Tensor, key_value: torch.Tensor) -> torch.Tensor:
        q = self.ln_q(query)
        kv = self.ln_kv(key_value)
        x, _ = self.cross_attn(q, kv, kv, need_weights=False)
        x = self.norm_ffn(x)
        return self.linear2(self.activation(self.linear1(x)))


class StudentTemporalBackbone(nn.Module):
    def __init__(
        self,
        feature_dim: int = 16,
        seq_len: int = 20,
        d_model: int = 64,
        nhead: int = 2,
        num_encoder_layers: int = 2,
        dim_feedforward: int = 128,
        num_query_tokens: int = 4,
        activation: str = "gelu",
        normalize_input: bool = True,
    ):
        super().__init__()
        self.feature_dim = int(feature_dim)
        self.seq_len = int(seq_len)
        self.d_model = int(d_model)
        self.flat_obs_dim = self.seq_len * self.feature_dim
        self.normalize_input = bool(normalize_input)
        self.num_query_tokens = int(num_query_tokens)

        self.register_buffer("obs_mean", torch.zeros(self.feature_dim, dtype=torch.float32))
        self.register_buffer("obs_std", torch.ones(self.feature_dim, dtype=torch.float32))

        act = activation_layer(activation)

        self.input_projection = nn.Sequential(
            nn.Linear(self.feature_dim, self.d_model),
            nn.LayerNorm(self.d_model),
            act(),
        )
        self.current_query_projection = nn.Sequential(
            nn.Linear(self.feature_dim, self.num_query_tokens * self.d_model),
            act(),
        )
        self.pos_embedding = nn.Parameter(torch.zeros(1, self.seq_len - 1, self.d_model))
        nn.init.trunc_normal_(self.pos_embedding, std=0.02)

        self.transformer_encoder = nn.ModuleList(
            [
                TransformerEncoderLayer(
                    d_model=self.d_model,
                    nhead=nhead,
                    dim_feedforward=dim_feedforward,
                    activation=activation,
                )
                for _ in range(num_encoder_layers)
            ]
        )
        self.cross_attention_block = CrossAttentionBlock(
            d_model=self.d_model,
            nhead=nhead,
            dim_feedforward=dim_feedforward,
            activation=activation,
        )
        self.ln_out = nn.LayerNorm(self.d_model)

    def _reshape_sequence(self, states: torch.Tensor) -> torch.Tensor:
        if states.dim() == 2:
            return states.view(states.shape[0], self.seq_len, self.feature_dim)
        return states

    def forward(self, states: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        x = self._reshape_sequence(states)
        if self.normalize_input:
            x = (x - self.obs_mean) / (self.obs_std + 1e-8)

        current_visible_obs = x[:, -1, :]
        query = self.current_query_projection(current_visible_obs)
        query = query.view(query.shape[0], self.num_query_tokens, self.d_model)

        past = x[:, :-1, :]   # ← 只取前 19 步
        kv = self.input_projection(past) + self.pos_embedding
        for layer in self.transformer_encoder:
            kv = layer(kv)

        attn_out = self.cross_attention_block(query=query, key_value=kv)
        attn_out = self.ln_out(attn_out)
        return attn_out.reshape(attn_out.shape[0], -1), current_visible_obs


class HexaPrivStudentModel(GaussianMixin, DeterministicMixin, Model):
    def __init__(
        self,
        observation_space,
        action_space,
        device,
        clip_actions: bool = False,
        clip_log_std: bool = True,
        min_log_std: float = -20.0,
        max_log_std: float = 2.0,
        reduction: str = "sum",
        seq_len: int = 20,
        visible_dim: int = 16,
        visible_feature_dim: int = 32,
        d_model: int = 64,
        privileged_bottleneck_dim: int = 16,
        privileged_feature_dim: int = 32,
        actor_hidden_dim: int = 128,
        critic_hidden_dim: int = 128,
        activation: str = "elu",
        transformer_activation: str = "gelu",
        nhead: int = 2,
        num_encoder_layers: int = 2,
        dim_feedforward: int = 128,
        num_query_tokens: int = 4,
        head_hidden_dim: int = 128,
        normalize_input: bool = True,
        **_: Any,
    ):
        Model.__init__(self, observation_space=observation_space, action_space=action_space, device=device)
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

        self.seq_len = int(seq_len)
        self.visible_dim = int(visible_dim)
        self.visible_feature_dim = int(visible_feature_dim)
        self.d_model = int(d_model)
        self.num_query_tokens = int(num_query_tokens)
        self.head_hidden_dim = int(head_hidden_dim)
        self.backbone_output_dim = self.d_model * self.num_query_tokens
        self.privileged_bottleneck_dim = int(privileged_bottleneck_dim)
        self.privileged_feature_dim = int(privileged_feature_dim)
        self.latent_dim = self.visible_feature_dim + self.privileged_feature_dim

        action_dim = self.num_actions
        act = activation_layer(activation)
        transformer_act = activation_layer(transformer_activation)

        self.visible_encoder = nn.Sequential(
            nn.Linear(self.visible_dim, self.visible_feature_dim),
            act(),
        )
        self.backbone = StudentTemporalBackbone(
            feature_dim=self.visible_dim,
            seq_len=self.seq_len,
            d_model=self.d_model,
            nhead=nhead,
            num_encoder_layers=num_encoder_layers,
            dim_feedforward=dim_feedforward,
            num_query_tokens=self.num_query_tokens,
            activation=transformer_activation,
            normalize_input=normalize_input,
        )
        self.bottleneck_head = nn.Sequential(
            nn.Linear(self.backbone_output_dim, self.head_hidden_dim),
            transformer_act(),
            nn.Linear(self.head_hidden_dim, self.privileged_bottleneck_dim),
        )
        self.privileged_expansion = nn.Sequential(
            nn.Linear(self.privileged_bottleneck_dim, self.privileged_feature_dim),
            act(),
        )
        self.actor_head = nn.Sequential(
            nn.Linear(self.latent_dim, actor_hidden_dim),
            act(),
            nn.Linear(actor_hidden_dim, actor_hidden_dim),
            act(),
            nn.Linear(actor_hidden_dim, action_dim),
        )
        self.critic_head = nn.Sequential(
            nn.Linear(self.latent_dim, critic_hidden_dim),
            act(),
            nn.Linear(critic_hidden_dim, critic_hidden_dim),
            act(),
            nn.Linear(critic_hidden_dim, 1),
        )
        self.log_std_parameter = nn.Parameter(torch.zeros(action_dim))

        print(f"[HexaPrivStudentModel] parameters={sum(p.numel() for p in self.parameters()):,}")

    def extract_features(self, states: torch.Tensor) -> dict[str, torch.Tensor]:
        attn_out, current_visible_obs = self.backbone(states)
        pred_bottleneck = self.bottleneck_head(attn_out)
        pred_privileged_feature = self.privileged_expansion(pred_bottleneck)
        visible_feature = self.visible_encoder(current_visible_obs)
        latent_feature = torch.cat([visible_feature, pred_privileged_feature], dim=1)
        return {
            "current_visible_obs": current_visible_obs,
            "visible_feature": visible_feature,
            "attn_out": attn_out,
            "pred_bottleneck": pred_bottleneck,
            "bottleneck_out": pred_bottleneck,
            "pred_privileged_feature": pred_privileged_feature,
            "cross_out": pred_privileged_feature,
            "latent_feature": latent_feature,
        }

    def act(self, inputs, role):
        if role == "policy":
            return GaussianMixin.act(self, inputs, role)
        if role == "value":
            return DeterministicMixin.act(self, inputs, role)
        raise ValueError(f"Unsupported role: {role}")

    def compute(self, inputs, role):
        features = self.extract_features(inputs["states"])
        latent_feature = features["latent_feature"]
        if role == "policy":
            return self.actor_head(latent_feature), self.log_std_parameter, {}
        if role == "value":
            return self.critic_head(latent_feature), {}
        raise ValueError(f"Unsupported role: {role}")

    def load_teacher_compatible_weights(
        self,
        teacher_state_dict: dict[str, torch.Tensor],
        strict_shapes: bool = False,
    ) -> dict[str, list[str]]:
        own_state = self.state_dict()
        prefixes = (
            "visible_encoder.",
            "privileged_expansion.",
            "actor_head.",
            "critic_head.",
            "log_std_parameter",
        )
        compatible = {}
        skipped = []

        for key, tensor in teacher_state_dict.items():
            if not key.startswith(prefixes):
                continue
            if key in own_state and own_state[key].shape == tensor.shape:
                compatible[key] = tensor
            else:
                skipped.append(key)

        self.load_state_dict(compatible, strict=False)
        return {"loaded": sorted(compatible.keys()), "skipped": sorted(skipped)}
