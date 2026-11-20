"""Deterministic random streams with explicit root-seed provenance."""

from __future__ import annotations

import hashlib
import random
from dataclasses import dataclass


@dataclass(frozen=True)
class SeedContext:
    root_seed: int

    def __post_init__(self) -> None:
        if not 0 <= self.root_seed < 2**64:
            raise ValueError("root_seed must be an unsigned 64-bit integer")

    def derive(self, namespace: str) -> int:
        if not namespace.strip():
            raise ValueError("seed namespace must be non-empty")
        payload = f"lexium-agro:{self.root_seed}:{namespace}".encode()
        return int.from_bytes(hashlib.sha256(payload).digest()[:8], "big")

    def random(self, namespace: str) -> random.Random:
        return random.Random(self.derive(namespace))
