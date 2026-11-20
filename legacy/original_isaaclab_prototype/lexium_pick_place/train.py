"""Launch script for training Schneider Lexium Cobot Pick-and-Place with RSL-RL."""

from __future__ import annotations

import argparse
import sys

from isaaclab.app import AppLauncher

# -----------------------------------------------------------------------------
# CLI Arguments Definition
# -----------------------------------------------------------------------------
parser = argparse.ArgumentParser(description="Train RL policy for Schneider Lexium Pick-and-Place.")
parser.add_argument("--num_envs", type=int, default=4096, help="Number of parallel simulation environments.")
parser.add_argument("--seed", type=int, default=42, help="Random seed for environment and policy.")
parser.add_argument("--max_iterations", type=int, default=None, help="Override maximum training iterations.")
parser.add_argument("--checkpoint", type=str, default=None, help="Path to checkpoint to resume training.")
parser.add_argument("--debug", action="store_true", default=False, help="Use fast debug runner settings (64 envs).")

# Append Isaac Lab AppLauncher CLI options (headless, device, etc.)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# Launch Omniverse / Isaac Sim application
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# -----------------------------------------------------------------------------
# Simulation Post-Launch Imports
# -----------------------------------------------------------------------------
import gymnasium as gym
from isaaclab_rl.rsl_rl import RslRlOnPolicyRunner, RslRlVecEnvWrapper  # VERIFY
from isaaclab_tasks.utils import parse_env_cfg  # VERIFY

import lexium_pick_place  # Registers 'Isaac-PickPlace-Lexium-v0'
from lexium_pick_place.agents import LexiumPickPlacePPORunnerCfg, LexiumPickPlacePPORunnerDebugCfg


def main():
    """Main training execution loop."""
    task_name = "Isaac-PickPlace-Lexium-v0"

    # Environment configuration parsing
    env_cfg = parse_env_cfg(
        task_name,
        device=args_cli.device,
        num_envs=64 if args_cli.debug else args_cli.num_envs,
    )

    # RSL-RL runner configuration
    agent_cfg = LexiumPickPlacePPORunnerDebugCfg() if args_cli.debug else LexiumPickPlacePPORunnerCfg()
    if args_cli.max_iterations is not None:
        agent_cfg.max_iterations = args_cli.max_iterations

    # Initialize gym environment and wrap for RSL-RL
    env = gym.make(task_name, cfg=env_cfg)
    env = RslRlVecEnvWrapper(env)  # VERIFY

    # Instantiate RSL-RL runner
    runner = RslRlOnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)  # VERIFY

    if args_cli.checkpoint:
        print(f"[INFO] Resuming training from checkpoint: {args_cli.checkpoint}")
        runner.load(args_cli.checkpoint)

    # Start policy optimization
    print(f"[INFO] Starting training for {task_name} with {env_cfg.scene.num_envs} environments...")
    runner.learn(num_learning_iterations=agent_cfg.max_iterations, init_at_random_ep_len=True)

    # Clean close
    env.close()
    simulation_app.close()


if __name__ == "__main__":
    main()
