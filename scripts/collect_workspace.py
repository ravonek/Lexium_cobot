#!/usr/bin/env python3
"""Read-only, privacy-aware capture of current Lexium Ubuntu workspace.

Usage:
  python3 scripts/collect_workspace.py --workspace ~/Desktop/adi/lexium_rl_workspace/lexium_pick_place --output ~/Downloads/LEXIUM_CURRENT_WORKSPACE_CODEX.zip
  python3 scripts/collect_workspace.py --workspace ... --output ... --include-large

No simulation or hardware motion is executed. Review resulting files before sharing.
"""
import argparse
import hashlib
import json
from pathlib import Path
from zipfile import ZipFile, ZIP_DEFLATED

INCLUDE_EXT={'.py','.json','.yaml','.yml','.toml','.md','.txt','.csv','.sh','.urdf','.xrdf','.usd','.usda','.usdc','.stl','.obj','.dae','.mtl','.png','.jpg','.jpeg','.npz','.npy','.xml','.ini','.cfg','.log'}
SKIP_DIR={'.git','.venv','venv','__pycache__','.cache','.pytest_cache','.mypy_cache','.ruff_cache','.idea','.vscode','node_modules','wandb','checkpoints','model_checkpoints'}
SKIP_SUBSTR=('secret','password','credential','private_key','id_rsa','token','.env','apikey','api_key','ssh','access_key')

def collect(root:Path,out:Path,large:bool=False):
    if not root.is_dir():
        raise SystemExit(f'Workspace not found: {root}')
    out.parent.mkdir(parents=True,exist_ok=True)
    skipped=[];written=[];max_bytes=(3*1024**3 if large else 100*1024**2)
    with ZipFile(out,'w',compression=ZIP_DEFLATED,compresslevel=6) as z:
        for path in sorted(root.rglob('*')):
            if path.is_symlink() or not path.is_file(): continue
            rel=path.relative_to(root)
            if any(x in SKIP_DIR for x in rel.parts):continue
            if any(w in path.name.lower() for w in SKIP_SUBSTR):
                skipped.append({'path':str(rel),'reason':'secret_like_name'});continue
            if path.suffix.lower() not in INCLUDE_EXT:
                skipped.append({'path':str(rel),'reason':'extension_not_selected'});continue
            try:s=path.stat().st_size
            except OSError:continue
            if s>max_bytes:
                skipped.append({'path':str(rel),'reason':f'large_{s}_bytes'});continue
            blob=path.read_bytes()
            z.writestr('workspace/'+rel.as_posix(),blob)
            written.append({'path':str(rel),'size':len(blob),'sha256':hashlib.sha256(blob).hexdigest()})
        z.writestr('COLLECTION_MANIFEST.json',json.dumps({'workspace_root':str(root),'files':written,'skipped':skipped,'warnings':['Inspect every file before public upload','Data may include personal paths and proprietary models']},indent=2))
    return written,skipped

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--workspace',required=True,type=Path);p.add_argument('--output',required=True,type=Path);p.add_argument('--include-large',action='store_true');a=p.parse_args()
    files,skip=collect(a.workspace.expanduser(),a.output.expanduser(),a.include_large)
    print(f'Created {a.output}: {len(files)} files, {len(skip)} skipped. Read COLLECTION_MANIFEST.json before sharing.')
