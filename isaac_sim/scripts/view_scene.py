"""Open a saved scene in a window and leave it open, camera already aimed.

    ./scripts/view_factory_scene.sh

Opening the file by hand works too, but Kit drops you wherever its
perspective camera last was and gives you a dark viewport if that is
nowhere near the building. This opens the file, frames it, and then just
runs the window, so the scene is on screen with no hunting.

Edits made in the window are edits to the file: File > Save writes back
to the same USD the experiment loads.
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/factory.yaml")
    parser.add_argument("--usd", default=None,
                        help="scene to open (default: factory.usd_path)")
    return parser.parse_args()


args = parse_args()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": False})

from isaacsim.core.utils.stage import open_stage  # noqa: E402
from isaacsim.core.utils.viewports import set_camera_view  # noqa: E402

from comfort_core import load_config, resolve_path  # noqa: E402
from comfort_core.factory_layout import build_layout  # noqa: E402


def main() -> int:
    cfg = load_config(common_path=args.config)
    path = Path(args.usd) if args.usd else resolve_path(cfg["factory"]["usd_path"])
    if not path.exists():
        print(f"[ERROR] {path} does not exist; run ./scripts/build_factory_scene.sh",
              flush=True)
        return 1

    print(f"  opening  : {path}", flush=True)
    if not open_stage(str(path)):
        print(f"[ERROR] could not open {path}", flush=True)
        return 1

    # A few frames before moving the camera: the viewport has to exist
    # first, and the stage has to have finished loading.
    for _ in range(60):
        simulation_app.update()

    layout = build_layout(cfg)
    set_camera_view(eye=[0.0, -layout.width * 1.15, max(layout.length, layout.width) * 0.75],
                    target=[0.0, 0.0, 1.0])
    print("  ready    : close the window to quit. Edits save back to this file.",
          flush=True)

    while simulation_app.is_running():
        simulation_app.update()
    return 0


if __name__ == "__main__":
    code = 1
    try:
        code = main()
    except Exception:
        import traceback
        traceback.print_exc()
    sys.stdout.flush()
    os._exit(code)
