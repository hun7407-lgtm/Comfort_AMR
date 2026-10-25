#!/usr/bin/env python3
"""Check that the ROS nodes would at least start.

    python3 scripts/check_nodes.py scripts/comfort_gate.py ...
    python3 scripts/check_nodes.py            # the four that matter

Plain Python 3: it must run without ROS installed, because the point is
to fail in two seconds rather than forty minutes.

What it checks
--------------
    the file parses
    every name it imports from comfort_core actually exists there

The second one is the one that earns its keep. A node that imports a
function from the wrong module dies the instant it starts, and the
symptom is not an error anybody sees: launch carries on, nothing
reaches the wheels, and the trial is recorded as a robot that sat at
its start position and deadlocked. Nine trials were lost that way to
one wrong import line.

ROS imports are not checked. They cannot be, from this interpreter.
"""

from __future__ import annotations

import ast
import importlib
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

DEFAULT = [
    "scripts/comfort_gate.py",
    "scripts/comfort_publisher.py",
    "scripts/nav2_goal.py",
    "scripts/human_tracker.py",
    "scripts/people_tracker.py",
]


def check(path: Path) -> list[str]:
    problems: list[str] = []
    try:
        tree = ast.parse(path.read_text(encoding="utf-8"))
    except SyntaxError as exc:
        return [f"{path}:{exc.lineno}: {exc.msg}"]

    for node in ast.walk(tree):
        if not isinstance(node, ast.ImportFrom) or not node.module:
            continue
        if not node.module.startswith("comfort_core"):
            continue
        try:
            module = importlib.import_module(node.module)
        except Exception as exc:  # noqa: BLE001
            problems.append(f"{path}:{node.lineno}: cannot import "
                            f"{node.module}: {exc}")
            continue
        for alias in node.names:
            if not hasattr(module, alias.name):
                problems.append(f"{path}:{node.lineno}: {node.module} has no "
                                f"{alias.name!r}")
    return problems


def main() -> int:
    targets = [Path(a) for a in sys.argv[1:]] or [PROJECT_ROOT / p for p in DEFAULT]
    problems: list[str] = []
    for path in targets:
        if not path.is_file():
            problems.append(f"{path}: not found")
            continue
        problems.extend(check(path))
    if problems:
        print("[check_nodes] " + "\n[check_nodes] ".join(problems),
              file=sys.stderr)
        return 1
    print(f"[check_nodes] {len(targets)} node(s) look importable")
    return 0


if __name__ == "__main__":
    sys.exit(main())
