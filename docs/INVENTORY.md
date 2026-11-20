# Phase 0 inventory and freeze

This document records the local import verification performed on 20 November
2026. It proves file integrity only; it does not prove any historical simulation,
learning, or physical-robot result.

## Handoff envelope

- Source ZIP: `LEXIUM_AGRO_CODEX_REPOSITORY_HANDOFF_2026-11-20.zip`
- ZIP SHA-256: `438c009d22c69d0269bed45adbd8de1f38642bc6ae6d25c2da43f9da2edb4a58`
- ZIP members: 215 (`SOURCE_MANIFEST.json` plus 214 declared files)
- Extracted source files checked against `SOURCE_MANIFEST.json`: 214/214
- Missing or changed source files before repository work: 0
- Initial immutable Git snapshot: `c74cca5`

Run the repeatable verification from the repository root:

```bash
.venv/bin/python scripts/verify_handoff.py
```

The verifier checks byte size and SHA-256 for every declared file, tests all
four nested ZIPs, and reports a conservative provenance class. The classes are
path-based labels, not claims about authorship, licensing, or scientific validity.

## Provenance boundaries

- `legacy/archives/` contains preserved source archives.
- `legacy/2026-11-10-starter/`, `legacy/extra_sources/`, and
  `legacy/original_isaaclab_prototype/` are historical material, not current code.
- `evidence/screenshots/` and `media/` are supplied visual/document artifacts.
- `evidence/measurements/` is transcription, not original raw simulator output.
- `assets/robot/references/` contains unverified reference assets.
- `src/`, `tests/`, `scripts/`, `configs/`, `docs/`, `ros2/`, `isaac_sim/`, and
  `isaac_lab/` began as the generated handoff scaffold.
- `thesis/` is a documentation artifact and must not be used as test evidence.

The binary report
`legacy/2026-11-10-starter/historical_reports/lexium_stage3c37_full_collision_triangles.npz`
is explicitly retained despite the general ignore rule for generated NumPy arrays.

## Privacy and publication check

A text-only high-risk credential-pattern scan of the extracted files found no
candidate assignment containing passwords, API keys, access tokens, private-key
headers, or AWS-style access keys. This narrow scan is not a security or licensing
clearance. Historical scripts contain personal absolute paths, and photos/CAD may
be private or proprietary. A public push remains unauthorized and requires a fresh
manual review.
