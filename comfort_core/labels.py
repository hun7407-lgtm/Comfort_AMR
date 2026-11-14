"""User-facing robot names: AMR 1 / AMR 2 / AMR 3.

Pure Python. Display only.

The internal IDs stay what they are, because each one is also a ROS
namespace, a TF prefix, an Isaac prim name, a Nav2 params file name, a
CSV `robot` value and a log tag:

    ""  / "amr1" / "AMR1"   the ego robot (no namespace; fleet_goals.py
                            keys its pose "amr1", metrics write
                            min_gap_amr1_m)
    "a1", "a2", "a3"        the fleet (fleet.amrs.<id>)

Renaming them would break every recorded batch and the launch chain, so
every NEW report, figure, table and log line maps them here instead.
"""

from __future__ import annotations

ROBOT_LABELS = {
    "": "AMR 1", "ego": "AMR 1", "amr1": "AMR 1", "AMR1": "AMR 1",
    "a1": "AMR 2", "a2": "AMR 3", "a3": "AMR 4",
}


def robot_label(rid) -> str:
    """'a1' -> 'AMR 2'. Unknown IDs are returned unchanged."""
    return ROBOT_LABELS.get(str(rid), str(rid))


def relabel(text: str) -> str:
    """Replace legacy names in a free-text label ('AMR1 vs a1 gap')."""
    import re
    return re.sub(r"\bAMR1\b|\ba[1-3]\b", lambda m: robot_label(m.group(0)), text)
