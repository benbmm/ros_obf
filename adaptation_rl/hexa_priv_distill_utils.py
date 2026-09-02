from __future__ import annotations

import os
from typing import Any

import gymnasium as gym
import torch
import yaml
import torch.nn as nn


def activation_layer(name: str):
    name = str(name).lower()
    if name == "elu":
        return nn.ELU
    if name == "relu":
        return nn.ReLU
    if name in {"silu", "swish"}:
        return nn.SiLU
    if name == "gelu":
        return nn.GELU
    if name == "tanh":
        return nn.Tanh
    raise ValueError(f"Unsupported activation: {name}")

def load_yaml(path: str) -> dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def checkpoint_agent_yaml_candidates(checkpoint_path: str) -> list[str]:
    checkpoint_path = os.path.abspath(checkpoint_path)
    checkpoint_dir = os.path.dirname(checkpoint_path)
    run_dir = os.path.dirname(checkpoint_dir)
    parent_dir = os.path.dirname(run_dir)
    return [
        os.path.join(checkpoint_dir, "params", "agent.yaml"),
        os.path.join(run_dir, "params", "agent.yaml"),
        os.path.join(parent_dir, "params", "agent.yaml"),
    ]


def checkpoint_agent_yaml_path(checkpoint_path: str) -> str:
    for path in checkpoint_agent_yaml_candidates(checkpoint_path):
        if os.path.exists(path):
            return path
    raise FileNotFoundError("Cannot find params/agent.yaml")


def load_agent_cfg_for_checkpoint(checkpoint_path: str, fallback_cfg: dict | None = None) -> dict:
    return load_yaml(checkpoint_agent_yaml_path(checkpoint_path))


def make_teacher_spaces(device: str = "cuda"):
    obs_space = gym.spaces.Box(low=-float("inf"), high=float("inf"), shape=(61,), dtype=float)
    act_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(6,), dtype=float)
    return obs_space, act_space


def make_student_spaces():
    obs_space = gym.spaces.Box(low=-float("inf"), high=float("inf"), shape=(20, 16), dtype=float)
    act_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(6,), dtype=float)
    return obs_space, act_space


def extract_checkpoint_state_dicts(checkpoint_path: str) -> tuple[dict[str, torch.Tensor], dict[str, torch.Tensor]]:
    ckpt = torch.load(checkpoint_path, map_location="cpu")
    policy_state = None
    scaler_state = None

    if isinstance(ckpt, dict):
        for key in ("policy", "models/policy", "model", "state_dict"):
            value = ckpt.get(key)
            if isinstance(value, dict):
                policy_state = value
                break
        models = ckpt.get("models")
        if policy_state is None and isinstance(models, dict) and isinstance(models.get("policy"), dict):
            policy_state = models["policy"]
        if policy_state is None:
            policy_state = ckpt

        for key in ("state_preprocessor", "_state_preprocessor", "preprocessor", "state_preprocessor_state_dict"):
            value = ckpt.get(key)
            if isinstance(value, dict):
                scaler_state = value
                break

    if policy_state is None:
        raise RuntimeError(f"Cannot find policy state_dict in checkpoint: {checkpoint_path}")

    return policy_state, scaler_state or {}


def _unwrap_preprocessor_output(value):
    return value[0] if isinstance(value, tuple) else value


def preprocess_states_with_agent(agent, states: torch.Tensor) -> torch.Tensor:
    preprocessor = getattr(agent, "_state_preprocessor", None) or getattr(agent, "state_preprocessor", None)
    if preprocessor is None:
        return states
    try:
        return _unwrap_preprocessor_output(preprocessor(states, train=False))
    except TypeError:
        try:
            return _unwrap_preprocessor_output(preprocessor(states, inverse=False, train=False))
        except TypeError:
            return _unwrap_preprocessor_output(preprocessor(states))


def _state_dict_from_obj(obj) -> dict:
    if obj is None:
        return {}
    if isinstance(obj, dict):
        return obj
    if hasattr(obj, "state_dict"):
        try:
            return obj.state_dict()
        except Exception:
            return {}
    return {}


def extract_scaler_mean_std(preprocessor_or_state: Any, expected_size: int = 61) -> tuple[torch.Tensor, torch.Tensor]:
    state = _state_dict_from_obj(preprocessor_or_state)
    mean = None
    var = None
    std = None

    for key, value in state.items():
        if not torch.is_tensor(value):
            continue
        name = str(key).lower()
        flat = value.detach().float().reshape(-1)
        if flat.numel() < expected_size:
            continue
        if mean is None and "mean" in name:
            mean = flat[:expected_size]
        if var is None and ("var" in name or "variance" in name):
            var = flat[:expected_size]
        if std is None and "std" in name:
            std = flat[:expected_size]

    for attr in ("mean", "running_mean", "_mean", "_running_mean"):
        value = getattr(preprocessor_or_state, attr, None)
        if mean is None and torch.is_tensor(value) and value.numel() >= expected_size:
            mean = value.detach().float().reshape(-1)[:expected_size]

    for attr in ("std", "running_std", "_std", "variance", "var", "running_var"):
        value = getattr(preprocessor_or_state, attr, None)
        if torch.is_tensor(value) and value.numel() >= expected_size:
            flat = value.detach().float().reshape(-1)[:expected_size]
            if "var" in attr or "variance" in attr:
                var = flat if var is None else var
            else:
                std = flat if std is None else std

    if mean is None:
        raise RuntimeError("Cannot extract mean from state preprocessor")
    if std is None:
        if var is None:
            raise RuntimeError("Cannot extract std or variance from state preprocessor")
        std = torch.sqrt(torch.clamp(var, min=1e-12))
    return mean, torch.clamp(std, min=1e-6)


def set_student_visible_normalization(student_model, teacher_mean_61: torch.Tensor, teacher_std_61: torch.Tensor):
    visible_mean = teacher_mean_61[: student_model.visible_dim].detach().float().to(student_model.device)
    visible_std = teacher_std_61[: student_model.visible_dim].detach().float().to(student_model.device)
    student_model.backbone.obs_mean.copy_(visible_mean)
    student_model.backbone.obs_std.copy_(visible_std)
    student_model.backbone.normalize_input = True
    return visible_mean, visible_std


def freeze_teacher_compatible_student_modules(student_model):
    for module in (
        student_model.visible_encoder,
        student_model.privileged_expansion,
        student_model.actor_head,
        student_model.critic_head,
    ):
        for p in module.parameters():
            p.requires_grad_(False)
    student_model.log_std_parameter.requires_grad_(False)
    for module in (student_model.backbone, student_model.bottleneck_head):
        for p in module.parameters():
            p.requires_grad_(True)
