#!/usr/bin/env python3
"""Create a local, self-contained Lexium Gemini project snapshot on Ubuntu.

Usage:
    python3 collect_full_lexium_from_ubuntu.py
    python3 collect_full_lexium_from_ubuntu.py --project /path/to/lexium_pick_place --downloads /path/to/reports

Requires Python stdlib only. Does not modify project files or run robot simulation.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import sys
from datetime import datetime, timezone
from pathlib import Path
from zipfile import ZipFile, ZIP_DEFLATED

HOME = Path.home()
DEFAULT_PROJECT = HOME / 'Desktop/adi/lexium_rl_workspace/lexium_pick_place'
DEFAULT_DOWNLOADS = HOME / 'Downloads'
DEFAULT_OUTPUT = DEFAULT_DOWNLOADS / 'LEXIUM_GEMINI_COMPLETE_LOCAL_2026-11-10.zip'

SKIP_DIRS = {
    '.git', '.venv', 'venv', 'env', '__pycache__', '.pytest_cache',
    '.mypy_cache', '.ruff_cache', '.cache', '.idea', '.vscode',
    'node_modules', 'wandb', 'runs', 'checkpoints', 'tensorboard',
    'cache', 'caches', 'logs', 'tmp', 'temp', '.tox', 'dist', 'build',
}
# Restrict to project-relevant source, configuration, robot assets, reports,
# and asset dependencies. Deliberately excludes tokens, key material, training
# checkpoints, and virtual environments.
PROJECT_SUFFIXES = {
    '.py', '.sh', '.md', '.txt', '.csv', '.yaml', '.yml', '.json',
    '.toml', '.ini', '.urdf', '.xrdf', '.xml', '.xacro',
    '.stl', '.obj', '.mtl', '.dae', '.usd', '.usda', '.usdc',
    '.npz', '.npy', '.png', '.jpg', '.jpeg', '.exr', '.ply', '.pcd',
}
REPORT_SUFFIXES = {'.log', '.json', '.txt', '.csv', '.npz', '.npy', '.png'}
MAX_SINGLE_FILE_BYTES = 300 * 1024 * 1024


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def should_skip(path: Path, root: Path) -> bool:
    rel = path.relative_to(root)
    if any(part in SKIP_DIRS or part.startswith('.') for part in rel.parts[:-1]):
        return True
    name = path.name.lower()
    if name.startswith('.') or name in {'secrets.yaml', 'secrets.yml'}:
        return True
    if any(k in name for k in ('password', 'apikey', 'api_key', 'credential', 'token_secret')):
        return True
    if path.suffix.lower() not in PROJECT_SUFFIXES:
        return True
    if path.is_symlink() or not path.is_file():
        return True
    return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project', type=Path, default=DEFAULT_PROJECT)
    parser.add_argument('--downloads', type=Path, default=DEFAULT_DOWNLOADS)
    parser.add_argument('--out', type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    project = args.project.expanduser().resolve()
    downloads = args.downloads.expanduser().resolve()
    out = args.out.expanduser().resolve()
    script_dir = Path(__file__).resolve().parent

    if not project.is_dir():
        parser.error('Project folder not found: ' + str(project))
    if not downloads.is_dir():
        parser.error('Reports/Downloads folder not found: ' + str(downloads))

    selections: dict[str, Path] = {}
    skipped_large = []

    def queue(src: Path, arcname: str):
        if not src.is_file() or src.is_symlink():
            return
        size = src.stat().st_size
        if size > MAX_SINGLE_FILE_BYTES:
            skipped_large.append({'file': str(src), 'size_bytes': size})
            return
        selections.setdefault(arcname, src)

    for root, dirs, files in os.walk(project, followlinks=False):
        root = Path(root)
        rel_root = root.relative_to(project)
        dirs[:] = [d for d in dirs if d not in SKIP_DIRS and not d.startswith('.')]
        for name in files:
            src = root / name
            if should_skip(src, project):
                continue
            rel = (rel_root / name).as_posix()
            queue(src, 'project/' + rel)

    for src in sorted(downloads.iterdir()):
        if not src.is_file() or src.is_symlink():
            continue
        if (re.match(r'^lexium_stage3c\d+', src.name, re.IGNORECASE)
                and src.suffix.lower() in REPORT_SUFFIXES):
            queue(src, 'reports_logs/' + src.name)

    # Handoff and starter archive instructions, if extracting both together.
    for name in (
        'LEXIUM_GEMINI_HANDOFF_2026-11-10.md',
        'README_FIRST.md',
        'CURRENT_STATUS_AND_MISSING_FILES.md',
    ):
        src = script_dir / name
        if src.is_file():
            queue(src, 'context/' + name)

    # If invoked directly from the starter ZIP extraction, carry forward its
    # older historical scripts/reports too, without superseding local files.
    for folder, destination in (
        ('historical_sources', 'historical_sources'),
        ('historical_reports', 'historical_reports'),
        ('historical_logs', 'historical_logs'),
    ):
        base = script_dir / folder
        if base.is_dir():
            for src in base.rglob('*'):
                if src.is_file():
                    queue(src, destination + '/' + src.relative_to(base).as_posix())

    checks = {
        'project/robot_cfg.py': 'Robot configuration',
        'project/assets/lexium_cobot.urdf': 'URDF model',
        'project/lexium_stage3c73_physx_two_finger_precontact_trial.py': 'Stage 73 source',
        'project/lexium_stage3c74_finger_cube_sensor_wiring.py': 'Stage 74 source',
        'project/lexium_stage3c75_finger_cube_positive_control.py': 'Stage 75 original',
        'project/lexium_stage3c75_diagnostic.py': 'Stage 75 diagnostics',
        'reports_logs/lexium_stage3c75_diagnostic.log': 'Stage 75 drift log',
    }
    missing = {label: name for name, label in checks.items() if name not in selections}
    xrdf = [n for n in selections if n.lower().endswith('.xrdf')]
    urdfs = [n for n in selections if n.lower().endswith('.urdf')]
    usd = [n for n in selections if n.lower().endswith(('.usd','.usda','.usdc'))]
    meshes = [n for n in selections if n.lower().endswith(('.stl','.dae','.obj'))]

    out.parent.mkdir(parents=True, exist_ok=True)
    with ZipFile(out, 'w', compression=ZIP_DEFLATED, compresslevel=6, allowZip64=True) as z:
        manifest = []
        for arc, src in sorted(selections.items()):
            z.write(src, arc)
            manifest.append({'archive_path': arc,
                             'size_bytes': src.stat().st_size,
                             'sha256': sha256(src)})
        meta = {
            'archive_type': 'Lexium complete local project snapshot',
            'built_utc': datetime.now(timezone.utc).isoformat(),
            'source_project': str(project),
            'source_downloads': str(downloads),
            'file_count': len(selections),
            'urdf_files': urdfs,
            'xrdf_files': xrdf,
            'usd_files': usd,
            'mesh_file_count': len(meshes),
            'missing_expected': missing,
            'skipped_over_300mb': skipped_large,
            'note': 'No Isaac/PhysX validation or hardware verification performed by this packaging script.'
        }
        z.writestr('MANIFEST.json', json.dumps({'metadata': meta, 'files': manifest}, ensure_ascii=False, indent=2))

    print('LEXIUM_GEMINI_LOCAL_ARCHIVE=', out)
    print('FILE_COUNT=', len(selections))
    print('URDF_COUNT=', len(urdfs))
    print('XRDF_COUNT=', len(xrdf), '(optional; 0 means no XRDF found)')
    print('USD_COUNT=', len(usd))
    print('MESH_COUNT=', len(meshes))
    print('SIZE_MB=', round(out.stat().st_size/1048576, 2))
    if missing:
        print('MISSING_EXPECTED_FILES:', json.dumps(missing, ensure_ascii=False, indent=2))
    if skipped_large:
        print('SKIPPED_OVER_300MB:', json.dumps(skipped_large, ensure_ascii=False, indent=2))
    print('ZIP_TEST=', 'PASS' if ZipFile(out).testzip() is None else 'FAIL')
    print('UPLOAD_THIS_ZIP_TO_GEMINI=', out)


if __name__ == '__main__':
    main()
