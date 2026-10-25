"""Read a hand-edited factory scene back into the config.

    ./scripts/read_factory_scene.sh            show what moved
    ./scripts/read_factory_scene.sh --apply    write it into the config

Open assets/scene/factory_bay.usd in Isaac Sim, drag machines around,
save, then run this. Every station is one Xform, so moving a machine is
one transform, and this reads that transform back as an explicit pose.
With --apply the station list in configs/factory.yaml is rewritten with
those poses, which makes the next build reproduce what you arranged.

Without --apply nothing is written; it just prints what changed.

It only reads USD, but pxr is not importable until Kit has started, so
it boots a headless app first. That is the twenty seconds.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import Dict, List, Tuple

PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))

def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/factory.yaml")
    parser.add_argument("--usd", default=None)
    parser.add_argument("--apply", action="store_true",
                        help="rewrite the station list in the config")
    return parser.parse_args()


args = parse_args()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": True})

from pxr import Gf, Usd, UsdGeom  # noqa: E402

from comfort_core import load_config, resolve_path  # noqa: E402
from comfort_core.factory_layout import build_layout  # noqa: E402

MOVED = 0.005  # metres / degrees below which a difference is just float noise


def read_poses(path: Path, root: str) -> Dict[str, Tuple[float, float, float]]:
    """Every station's world pose in the file, as (x, y, yaw degrees)."""
    stage = Usd.Stage.Open(str(path))
    if stage is None:
        raise SystemExit(f"[ERROR] could not open {path}")
    parent = stage.GetPrimAtPath(f"{root}/Stations")
    if not parent:
        raise SystemExit(f"[ERROR] {path} has no {root}/Stations")

    out: Dict[str, Tuple[float, float, float]] = {}
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    for prim in parent.GetChildren():
        m = cache.GetLocalToWorldTransform(prim)
        t = m.ExtractTranslation()
        # The modules are only ever turned about z, so the rotation is
        # recoverable from where the local +x axis ended up.
        axis = m.TransformDir(Gf.Vec3d(1.0, 0.0, 0.0))
        angle = math.degrees(math.atan2(axis[1], axis[0]))
        # The stored angle is the local frame's; yaw is the front, which
        # is +y local, so ninety degrees further round.
        out[prim.GetName()] = (float(t[0]), float(t[1]), angle + 90.0)
    return out


def station_lines(cfg, poses: Dict[str, Tuple[float, float, float]]) -> List[str]:
    """The `stations:` block, with every pose written out explicitly."""
    lines = ["  stations:"]
    by_band: Dict[str, List[dict]] = {}
    for item in cfg["equipment"]["stations"]:
        by_band.setdefault(str(item["band"]), []).append(item)

    for band, items in by_band.items():
        lines.append(f"    # {band}")
        for item in items:
            name = str(item.get("name", item["module"]))
            x, y, yaw = poses.get(name, (float(item["x"]), None, None))
            parts = [f"name: {name}", f"module: {item['module']}",
                     f"band: {band}", f"x: {x:.3f}"]
            if y is not None:
                parts.append(f"y: {y:.3f}")
                parts.append(f"yaw_deg: {yaw:.1f}")
            if "align" in item:
                parts.append(f"align: {item['align']}")
            if "side" in item:
                parts.append(f"side: {item['side']}")
            if "worker" in item:
                parts.append(f"worker: {str(item['worker']).lower()}")
            lines.append("    - {" + ", ".join(parts) + "}")
    return lines


def rewrite(config_path: Path, lines: List[str]) -> None:
    """Replace the station list, keeping every comment above it.

    The list is the last thing in the file on purpose: everything that
    explains the layout stays where it is, and only the numbers move.
    """
    text = config_path.read_text()
    marker = "\n  stations:\n"
    if marker not in text:
        raise SystemExit(f"[ERROR] {config_path} has no `stations:` list to replace")
    head = text[:text.index(marker)]
    config_path.write_text(head + "\n" + "\n".join(lines) + "\n")


def main() -> int:
    cfg = load_config(common_path=args.config)
    path = Path(args.usd) if args.usd else resolve_path(cfg["factory"]["usd_path"])
    if not path.exists():
        print(f"[ERROR] {path} does not exist", flush=True)
        return 1

    root = str(cfg["factory"]["root_prim"])
    poses = read_poses(path, root)
    layout = build_layout(cfg)

    print(f"\n=== {path.name} vs {args.config} ===", flush=True)
    moved = 0
    for station in layout.stations:
        pose = poses.get(station.name)
        if pose is None:
            print(f"  {station.name:<10s} not in the file", flush=True)
            continue
        x, y, yaw = pose
        dx, dy = x - station.x, y - station.y
        dyaw = (yaw - math.degrees(station.yaw) + 180.0) % 360.0 - 180.0
        if max(abs(dx), abs(dy), abs(dyaw)) < MOVED:
            continue
        moved += 1
        print(f"  {station.name:<10s} moved  dx {dx:+.3f}  dy {dy:+.3f}  "
              f"dyaw {dyaw:+.1f} deg", flush=True)
    extra = sorted(set(poses) - {s.name for s in layout.stations})
    for name in extra:
        x, y, yaw = poses[name]
        print(f"  {name:<10s} is in the file but not in the config "
              f"(x {x:+.2f}, y {y:+.2f}); add it by hand", flush=True)

    if moved == 0 and not extra:
        print("  nothing moved\n", flush=True)
        return 0

    lines = station_lines(cfg, poses)
    if args.apply:
        rewrite(resolve_path(args.config), lines)
        print(f"\n  written  : {args.config}", flush=True)
        print("  rebuild  : ./scripts/build_factory_scene.sh\n", flush=True)
    else:
        print("\n  --apply would write:\n", flush=True)
        for line in lines:
            print(f"  {line}", flush=True)
        print("", flush=True)
    return 0


if __name__ == "__main__":
    code = 1
    try:
        code = main()
    except Exception:
        import traceback
        traceback.print_exc()
    sys.stdout.flush()
    import os
    os._exit(code)
