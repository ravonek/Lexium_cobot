import json
import logging

import pytest

from lexium_agro.observability import JsonFormatter


def test_json_log_keeps_structured_fields() -> None:
    record = logging.LogRecord("lexium", logging.INFO, __file__, 1, "ready", (), None)
    record.event_fields = {"run_id": "offline-1", "seed": 7}
    payload = json.loads(JsonFormatter().format(record))
    assert payload["message"] == "ready"
    assert payload["fields"] == {"run_id": "offline-1", "seed": 7}


def test_non_mapping_fields_fail_loudly() -> None:
    record = logging.LogRecord("lexium", logging.INFO, __file__, 1, "bad", (), None)
    record.event_fields = "not-a-mapping"
    with pytest.raises(TypeError, match="dictionary"):
        JsonFormatter().format(record)
