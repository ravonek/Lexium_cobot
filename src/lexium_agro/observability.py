"""Small JSON-lines formatter for auditable offline and simulation logs."""

from __future__ import annotations

import json
import logging
from datetime import datetime, timezone
from typing import Any


class JsonFormatter(logging.Formatter):
    def format(self, record: logging.LogRecord) -> str:
        event: dict[str, Any] = {
            "timestamp": datetime.fromtimestamp(record.created, tz=timezone.utc).isoformat(),
            "level": record.levelname,
            "logger": record.name,
            "message": record.getMessage(),
        }
        fields = getattr(record, "event_fields", None)
        if fields is not None:
            if not isinstance(fields, dict):
                raise TypeError("event_fields must be a dictionary")
            event["fields"] = fields
        if record.exc_info:
            event["exception"] = self.formatException(record.exc_info)
        return json.dumps(event, sort_keys=True, separators=(",", ":"), default=str)
