"""Colours that have to agree across the simulator and the recordings.

Pure Python. A fleet AMR wears a stripe in the simulator and appears
as a labelled destination in the costmap video; if the two were picked
separately they would drift apart, and a reader could not tell from
the camera which job list a vehicle was running. One table, read by
both. Nothing here affects a trial: colour is drawing only.
"""

from __future__ import annotations

from typing import Dict, Tuple

RGB = Tuple[float, float, float]

# Distinct from the human-state palette (blue / yellow / red) and from
# AMR1's light blue, so a frame never has two meanings for
# one hue.
FLEET_COLOURS: Dict[str, RGB] = {
    "a1": (0.95, 0.50, 0.10),   # orange
    "a2": (0.60, 0.20, 0.80),   # violet
    "a3": (0.10, 0.70, 0.45),   # green
}
_FALLBACK: RGB = (0.85, 0.85, 0.85)

# AMR1: its start, goal and route markers, in the
# simulator and in the costmap video alike.
MISSION_COLOUR: RGB = (0.35, 0.78, 1.00)


def fleet_colour(name: str) -> RGB:
    """0..1 RGB for fleet AMR `name`; grey for one the table does not know."""
    return FLEET_COLOURS.get(str(name), _FALLBACK)


def fleet_rgb255(name: str) -> Tuple[int, int, int]:
    """The same colour as 8-bit RGB, for OpenCV drawing."""
    r, g, b = fleet_colour(name)
    return (int(round(r * 255)), int(round(g * 255)), int(round(b * 255)))


def mission_rgb255() -> Tuple[int, int, int]:
    r, g, b = MISSION_COLOUR
    return (int(round(r * 255)), int(round(g * 255)), int(round(b * 255)))
