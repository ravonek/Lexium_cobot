"""Registration of the Schneider Lexium Cobot Pick-and-Place Gym Environment."""

import gymnasium as gym

# Follow Isaac Lab 3.0 registration conventions
gym.register(
    id="Isaac-PickPlace-Lexium-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",  # VERIFY
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": "lexium_pick_place.pick_place_env_cfg:LexiumPickPlaceEnvCfg",
        "rsl_rl_cfg_entry_point": "lexium_pick_place.agents:LexiumPickPlacePPORunnerCfg",
    },
)
