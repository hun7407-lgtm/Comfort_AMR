"""YAML configuration loading for the Comfort_AMR experiments.

The experiment design requires that B2 and B3 differ in exactly one
thing: the Comfort Space Policy. That is enforced here by construction.
`configs/common.yaml` holds everything shared, a scenario file holds only
its policy, and `load_config` refuses to merge a scenario file that
reaches into a section it has no business touching.
"""

from __future__ import annotations

import copy
import os
from pathlib import Path
from typing import Any, Dict, Optional

import yaml

# Sections a scenario config is allowed to define. Anything else would
# make B2 and B3 differ in more than the policy, which invalidates the
# comparison. See docs/EXPERIMENT.md section 2.
SCENARIO_ALLOWED_SECTIONS = frozenset({"scenario", "comfort_space"})


def project_root() -> Path:
    """Absolute path to the repository root."""
    return Path(__file__).resolve().parent.parent


def resolve_path(value: str | os.PathLike) -> Path:
    """Turn a config path into an absolute one.

    Relative paths are resolved against the project root, never against
    the current working directory, so a script behaves the same however
    it was launched.
    """
    p = Path(value)
    return p if p.is_absolute() else (project_root() / p)


def load_yaml(path: str | os.PathLike) -> Dict[str, Any]:
    path = resolve_path(path)
    if not path.is_file():
        raise FileNotFoundError(f"config file not found: {path}")
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if data is None:
        return {}
    if not isinstance(data, dict):
        raise ValueError(f"config must be a mapping at the top level: {path}")
    return data


def deep_merge(base: Dict[str, Any], overlay: Dict[str, Any]) -> Dict[str, Any]:
    """Recursively merge `overlay` onto a copy of `base`.

    Lists are replaced wholesale, not concatenated: a scenario that sets
    a list means that list, not "these in addition to the defaults".
    """
    out = copy.deepcopy(base)
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = deep_merge(out[key], value)
        else:
            out[key] = copy.deepcopy(value)
    return out


def load_with_base(path: str | os.PathLike, _seen: set | None = None) -> Dict[str, Any]:
    """Load a YAML config, following a top-level `base:` key if present.

    Lets a variant config change three values without copying the whole
    of common.yaml, which would drift out of step the moment common.yaml
    changes.

        base: configs/common.yaml
        corridor:
          width: 1.2
    """
    resolved = resolve_path(path).resolve()
    _seen = _seen or set()
    if resolved in _seen:
        raise ValueError(f"config `base:` chain loops back to {resolved}")
    _seen.add(resolved)

    data = load_yaml(resolved)
    base = data.pop("base", None)
    if base is None:
        return data
    return deep_merge(load_with_base(base, _seen), data)


def load_config(
    scenario: Optional[str] = None,
    common_path: str | os.PathLike = "configs/common.yaml",
) -> Dict[str, Any]:
    """Load `common.yaml`, optionally overlaid with one scenario config.

    Args:
        scenario: "b1", "b2", "b3", or a path to a scenario YAML. None loads
            the common config alone, which is what the STEP 3 and STEP 4
            hardware checks need.
        common_path: override for the shared config, for tests.

    Raises:
        ValueError: if the scenario config defines a section outside
            SCENARIO_ALLOWED_SECTIONS. That would silently break the
            B2/B3 comparison, so it is an error rather than a warning.
    """
    config = load_with_base(common_path)

    if scenario is None:
        config["scenario"] = {"name": "common", "policy": None}
        return config

    if scenario in ("b1", "b2", "b3"):
        scenario_path = resolve_path(f"configs/scenario_{scenario}.yaml")
    else:
        scenario_path = resolve_path(scenario)

    overlay = load_with_base(scenario_path)

    illegal = set(overlay) - SCENARIO_ALLOWED_SECTIONS
    if illegal:
        raise ValueError(
            f"{scenario_path.name} defines section(s) {sorted(illegal)}, which "
            f"belong in configs/common.yaml. B2 and B3 must differ only in "
            f"{sorted(SCENARIO_ALLOWED_SECTIONS)}. See docs/EXPERIMENT.md."
        )

    return deep_merge(config, overlay)


def nav2_map(config: Dict[str, Any]) -> str:
    """Which map this config navigates on.

    Nav2's static layer, the Comfort Space mask and anything else that
    has to line up with the floor all have to agree on one file. A
    config may name its own -- the 3D variants navigate on the map SLAM
    built rather than the one generated from the layout -- and otherwise
    it is whatever scripts/generate_map.py writes for that scene.
    """
    named = config.get("nav2", {}).get("map")
    if named:
        return str(named)
    return ("configs/nav2/factory_map.yaml" if "factory" in config
            else "configs/nav2/corridor_map.yaml")
