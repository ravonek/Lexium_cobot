# Contributing to Lexium Agro

Thank you for helping improve the project. Contributions must preserve its safety
and evidence boundaries.

## Before opening a change

1. Read `PROJECT_SPEC.md`, `docs/STATUS_AND_PROVENANCE.md`, and `docs/SAFETY.md`.
2. Do not add credentials, controller endpoints, private calibration data, or
   unreviewed live-hardware commands.
3. Keep historical files under `legacy/` unchanged. Import or adapt them through a
   reviewable diff rather than rewriting the record.
4. Label synthetic, simulated, transcribed, and physical data explicitly.
5. Never report a test, training run, or experiment as successful unless it was
   invoked and its output inspected.

## Development setup

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -e '.[dev]'
.venv/bin/python scripts/verify_handoff.py
.venv/bin/python -m ruff check src tests scripts
.venv/bin/python -m mypy src/lexium_agro
.venv/bin/python -m pytest -q
```

## Pull requests

- Keep changes small and focused.
- State which commands were actually run and paste concise results.
- Describe data provenance, units, frames, seeds, and acceptance criteria.
- Separate simulator results from physical measurements.
- For derived percentages, provide numerator and denominator.
- Do not weaken the disabled hardware backend or safety defaults.

Changes that can reach real actuators require a separate safety-reviewed process and
are outside the default contribution workflow.
