"""Playback / evaluation script for Schneider Lexium Cobot Pick-and-Place."""

from __future__ import annotations

import argparse
import os

from isaaclab.app import AppLauncher

# -----------------------------------------------------------------------------
# CLI Arguments Definition
# -----------------------------------------------------------------------------
parser = argparse.ArgumentParser(description="Evaluate a trained policy for Schneider Lexium Cobot.")
parser.add_argument("--num_envs", type=int, default=16, help="Number of evaluation environments in viewport.")
parser.add_argument("--checkpoint", type=str, required=True, help="Path to trained model checkpoint (.pt).")
parser.add_argument("--use_keyboard", action="store_true", default=False, help="Enable keyboard reset triggers.")

# Append AppLauncher CLI options
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# Launch simulation app in GUI mode (unless --headless is explicitly passed)
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# -----------------------------------------------------------------------------
# Simulation Post-Launch Imports
# -----------------------------------------------------------------------------
import gymnasium as gym
import torch
from isaaclab_rl.rsl_rl import RslRlOnPolicyRunner, RslRlVecEnvWrapper  # VERIFY
from isaaclab_tasks.utils import parse_env_cfg  # VERIFY

import lexium_pick_place
from lexium_pick_place.agents import LexiumPickPlacePPORunnerCfg


def main():
    """Run trained policy inference inside viewport."""
    task_name = "Isaac-PickPlace-Lexium-v0"

    env_cfg = parse_env_cfg(task_name, device=args_cli.device, num_envs=args_cli.num_envs)
    agent_cfg = LexiumPickPlacePPORunnerCfg()

    env = gym.make(task_name, cfg=env_cfg)
    env = RslRlVecEnvWrapper(env)  # VERIFY

    runner = RslRlOnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)  # VERIFY

    print(f"[INFO] Loading checkpoint: {args_cli.checkpoint}")
    runner.load(args_cli.checkpoint)
    policy = runner.get_inference_policy(device=env.unwrapped.device)  # VERIFY

    obs, _ = env.get_observations()  # VERIFY

    print("[INFO] Starting evaluation loop. Press Ctrl+C or close viewport to exit.")
    while simulation_app.is_running():
        with torch.inference_mode():
            actions = policy(obs)
            obs, rewards, dones, infos = env.step(actions)

    env.close()
    simulation_app.close()


if __name__ == "__main__":
    main()
