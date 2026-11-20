"""Convert Lexium URDF with Isaac Lab 3.0-EA URDF importer (PhysX).

From the IsaacLab repository root (with configured Isaac Sim 6.1):
    uv run --extra isaacsim python /path/to/lexium_pick_place/convert_urdf.py

Only uses project-relative paths derived from __file__. Does not launch RL.
"""
from __future__ import annotations

from pathlib import Path

from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg

ASSETS = Path(__file__).resolve().parent / "assets"
URDF = ASSETS / "lexium_cobot.urdf"
OUTPUT = ASSETS / "usd"


def main() -> None:
    if not URDF.is_file():
        raise FileNotFoundError(f"Place the robot URDF here before converting: {URDF}")
    OUTPUT.mkdir(parents=True, exist_ok=True)
    cfg = UrdfConverterCfg(
        asset_path=str(URDF),
        usd_dir=str(OUTPUT),
        usd_file_name="lexium_cobot.usd",
        force_usd_conversion=True,
        fix_base=True,
        merge_fixed_joints=False,
        self_collision=True,
        collision_type="Convex Decomposition",
        replace_cylinders_with_capsules=False,
        physics_variant="physx",
    )
    result = UrdfConverter(cfg)
    print("[LEXIUM] Converted USD:", result.usd_path)
    print("[LEXIUM] Use the *actual* converter.usd_path in ArticulationCfg.spawn=UsdFileCfg(...).")
    print("[LEXIUM] Preserve all generated USD layers/payloads under assets/usd/.")


if __name__ == "__main__":
    main()
