"""The generated map's grid, for anything that has to line up with it.

Pure Python. The Comfort Space mask and the keep-right mask are both
drawn on the map's own grid so a mask cell and a costmap cell line up
exactly; this is where both read that grid from.
"""

from __future__ import annotations

from pathlib import Path

import yaml


def read_map_geometry(map_yaml: Path):
    """Take the mask's extent and resolution from the generated map.

    Sharing the map's grid means a mask cell and a costmap cell line up
    exactly, so the painted radius is the radius Nav2 applies.
    """
    meta = yaml.safe_load(map_yaml.read_text())
    image = (map_yaml.parent / meta["image"]).resolve()

    # Minimal PGM header parse: magic, width, height, maxval, skipping
    # comments. Avoids pulling in an image library.
    data = image.read_bytes()
    fields, i = [], 0
    while len(fields) < 4:
        if data[i:i + 1] == b"#":
            while data[i:i + 1] != b"\n":
                i += 1
            i += 1
            continue
        j = i
        while data[j:j + 1] not in (b" ", b"\n", b"\t"):
            j += 1
        token = data[i:j]
        if token:
            fields.append(token)
        i = j + 1

    return {
        "width": int(fields[1]),
        "height": int(fields[2]),
        "resolution": float(meta["resolution"]),
        "origin": [float(v) for v in meta["origin"]],
    }
