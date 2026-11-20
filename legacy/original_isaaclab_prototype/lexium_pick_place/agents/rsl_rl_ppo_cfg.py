"""RSL-RL PPO runner configurations for Schneider Lexium Pick-and-Place."""

from isaaclab.utils import configclass  # VERIFY
from isaaclab_rl.rsl_rl import (  # VERIFY
    RslRlOnPolicyRunnerCfg,
    RslRlPpoActorCriticCfg,
    RslRlPpoAlgorithmCfg,
)


@configclass
class LexiumPickPlacePPORunnerCfg(RslRlOnPolicyRunnerCfg):
    """PPO runner configuration targeting RSL-RL 5.x."""

    num_steps_per_env = 24
    max_iterations = 2500
    save_interval = 100
    experiment_name = "lexium_pick_place"
    empirical_normalization = False

    policy = RslRlPpoActorCriticCfg(
        init_noise_std=1.0,
        actor_hidden_dims=[256, 128, 64],
        critic_hidden_dims=[256, 128, 64],
        activation="elu",
    )

    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.005,
        num_learning_epochs=5,
        num_mini_batches=4,
        learning_rate=1.0e-4,
        schedule="adaptive",
        gamma=0.99,
        lam=0.95,
        desired_kl=0.01,
        max_grad_norm=1.0,
    )


@configclass
class LexiumPickPlacePPORunnerDebugCfg(LexiumPickPlacePPORunnerCfg):
    """Fast debug profile (for 64 environments)."""

    max_iterations = 200
    save_interval = 25
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.005,
        num_learning_epochs=5,
        num_mini_batches=2,
        learning_rate=2.0e-4,
        schedule="adaptive",
        gamma=0.99,
        lam=0.95,
        desired_kl=0.01,
        max_grad_norm=1.0,
    )
