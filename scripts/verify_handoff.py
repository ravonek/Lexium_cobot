#!/usr/bin/env python3
"""Verify the immutable handoff manifest and bundled ZIP integrity.

This is an offline provenance check. It does not import Isaac, ROS, or hardware
drivers and it never modifies the repository.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from collections import Counter
from pathlib import Path
from typing import Any
from zipfile import BadZipFile, ZipFile


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def classify(path: str) -> str:
    """Return a provenance class without implying experimental validity."""
    if path.startswith("legacy/archives/"):
        return "preserved_source_archive"
    if path.startswith(("legacy/2026-11-10-starter/", "legacy/extra_sources/")):
        return "preserved_historical_material"
    if path.startswith("legacy/original_isaaclab_prototype/"):
        return "preserved_legacy_prototype"
    if path.startswith(("media/", "evidence/screenshots/")):
        return "user_supplied_media_or_screenshot"
    if path.startswith("evidence/measurements/"):
        return "transcribed_evidence_not_raw"
    if path.startswith("assets/robot/references/"):
        return "unverified_reference_asset"
    if path.startswith("thesis/"):
        return "documentation_artifact"
    return "handoff_scaffold"


def verify(root: Path, manifest_path: Path) -> dict[str, Any]:
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    missing: list[str] = []
    changed_source: list[dict[str, Any]] = []
    changed_scaffold: list[dict[str, Any]] = []
    classes: Counter[str] = Counter()
    archives: list[dict[str, Any]] = []

    for item in manifest["files"]:
        relative = item["path"]
        classes[classify(relative)] += 1
        path = root / relative
        if not path.is_file():
            missing.append(relative)
            continue
        actual_size = path.stat().st_size
        actual_hash = sha256(path)
        if actual_size != item["bytes"] or actual_hash != item["sha256"]:
            mismatch = {
                "path": relative,
                "expected_bytes": item["bytes"],
                "actual_bytes": actual_size,
                "expected_sha256": item["sha256"],
                "actual_sha256": actual_hash,
            }
            target = changed_scaffold if classify(relative) == "handoff_scaffold" else changed_source
            target.append(mismatch)

        if relative.startswith("legacy/archives/") and path.suffix == ".zip":
            try:
                with ZipFile(path) as archive:
                    bad_member = archive.testzip()
                    member_count = len(archive.infolist())
            except BadZipFile:
                bad_member = "<invalid zip>"
                member_count = 0
            archives.append(
                {
                    "path": relative,
                    "sha256": actual_hash,
                    "members": member_count,
                    "integrity": "ok" if bad_member is None else "failed",
                    "first_bad_member": bad_member,
                }
            )

    return {
        "purpose": "offline_handoff_integrity_only",
        "manifest": manifest_path.relative_to(root).as_posix(),
        "manifest_declared_files": manifest["file_count_excluding_manifest"],
        "verified_files": manifest["file_count_excluding_manifest"] - len(missing),
        "missing": missing,
        "changed_source_artifacts": changed_source,
        "changed_scaffold": changed_scaffold,
        "provenance_classes": dict(sorted(classes.items())),
        "archives": archives,
        "valid": not missing
        and not changed_source
        and all(item["integrity"] == "ok" for item in archives),
        "warning": (
            "Integrity is not experimental validation; current Stage 6/Phase 10 "
            "sources and raw reports are still missing."
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--manifest", type=Path, default=Path("SOURCE_MANIFEST.json"))
    args = parser.parse_args()

    root = args.root.resolve()
    manifest = args.manifest if args.manifest.is_absolute() else root / args.manifest
    report = verify(root, manifest)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["valid"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
