from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml


ROOT = Path(__file__).resolve().parents[1]


def load_config(path: str | Path = ROOT / "configs" / "baseline.yaml") -> dict[str, Any]:
    path = Path(path).resolve()
    config = yaml.safe_load(path.read_text())
    config["_config_path"] = str(path)
    config["_root"] = str(ROOT)
    return config


def rooted(config: dict[str, Any], value: str | Path) -> Path:
    path = Path(value)
    return path if path.is_absolute() else Path(config["_root"]) / path

