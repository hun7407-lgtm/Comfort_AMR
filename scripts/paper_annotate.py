#!/usr/bin/env python3
"""Annotations and panels for the paper captures. Plain Python 3 + Pillow.

    python3 scripts/paper_annotate.py width  <ortho.png> <out.png>
    python3 scripts/paper_annotate.py extent <ortho.png> <out.png>
    python3 scripts/paper_annotate.py robots <ortho.png> <out.png>
    python3 scripts/paper_annotate.py banner <in.png> <out.png> --text "Comfort Gate Hold" [--sub "..."]
    python3 scripts/paper_annotate.py grid   <out.png> <a.png> <b.png> ... --cols 2 --titles "C1" "C2" ...

Every number drawn is read, never typed: corridor width from the scene
config, Comfort extents from the sidecar JSON that
isaac_sim/scripts/paper_shots.py wrote beside the image (the shapes it
drew), the effective extent e from comfort_core.feasibility.extent_of on
the same config. Orthographic images carry their metres -> pixel mapping
in that sidecar, so marks land where the geometry is.

The raw image is never overwritten; output always goes to a new file.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
FONT_B = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
INK = (20, 20, 20)
ACCENT = (213, 94, 0)
BLUE = (0, 114, 178)


def font(size, bold=False):
    try:
        return ImageFont.truetype(FONT_B if bold else FONT, size)
    except OSError:
        return ImageFont.load_default()


def meta_of(png: Path) -> dict:
    return json.loads(png.with_suffix(".json").read_text())


def px(meta, x, y):
    ox, oy = meta["ortho_origin_px"]
    k = meta["px_per_m"]
    return ox + x * k, oy - y * k


def label_box(d, xy, text, f, fill=INK, bg=(255, 255, 255, 215), anchor="mm"):
    l, t, r, b = d.textbbox(xy, text, font=f, anchor=anchor)
    d.rectangle((l - 6, t - 4, r + 6, b + 4), fill=bg)
    d.text(xy, text, font=f, fill=fill, anchor=anchor)


def arrow(d, a, b, colour, w=4, head=16, both=False):
    d.line([a, b], fill=colour, width=w)
    for p, q in ((b, a), (a, b)) if both else ((b, a),):
        ang = math.atan2(p[1] - q[1], p[0] - q[0])
        for s in (-0.45, 0.45):
            d.line([p, (p[0] - head * math.cos(ang + s), p[1] - head * math.sin(ang + s))],
                   fill=colour, width=w)


def width(src: Path, out: Path, x_at: float = -3.5):
    from comfort_core.config import load_config
    from comfort_core.layout import corridor_bounds
    m = meta_of(src)
    cfg = load_config(common_path=m["config"])
    b = corridor_bounds(cfg)
    im = Image.open(src).convert("RGBA")
    ov = Image.new("RGBA", im.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(ov)
    a, c = px(m, x_at, b.y_min), px(m, x_at, b.y_max)
    arrow(d, a, c, ACCENT, w=5, both=True)
    label_box(d, (a[0] + 150, (a[1] + c[1]) / 2), f"W = {b.y_max - b.y_min:.1f} m", font(40, True), ACCENT)
    Image.alpha_composite(im, ov).convert("RGB").save(out)
    return {"W": round(b.y_max - b.y_min, 3)}


def extent(src: Path, out: Path):
    """Person centre, the Comfort Space boundary (un-grown) and e toward the passage."""
    from comfort_core import feasibility as fz
    from comfort_core.config import load_config
    from comfort_core.size_sweep import parse_condition
    m = meta_of(src)
    cfg = load_config(scenario=m["scenario"], common_path=m["config"])
    kind_val = parse_condition(m["scenario"] or "")
    kind = "static" if kind_val and kind_val[0] == "static" else "b3o"
    e = fz.extent_of(cfg, kind, kind_val[1], "a")
    hx, hy = cfg["humans"]["a"]["position"]
    shape = m["comfort_shapes_ungrown"]["a"]
    im = Image.open(src).convert("RGBA")
    ov = Image.new("RGBA", im.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(ov)
    cx, cy = shape["centre"]
    c, s = math.cos(shape["heading_rad"]), math.sin(shape["heading_rad"])
    pts = []
    for i in range(240):
        t = 2 * math.pi * i / 240
        u, v = shape["along"] * math.cos(t), shape["across"] * math.sin(t)
        pts.append(px(m, cx + u * c - v * s, cy + u * s + v * c))
    d.line(pts + [pts[0]], fill=BLUE + (255,), width=5)
    hp = px(m, hx, hy)
    d.ellipse((hp[0] - 9, hp[1] - 9, hp[0] + 9, hp[1] + 9), fill=INK)
    tip = px(m, hx, hy - e)          # passage is toward -y in every Stage 1 scene
    arrow(d, hp, tip, ACCENT, w=6, head=20)
    k = m["px_per_m"]
    label_box(d, (tip[0] + 18, (hp[1] + tip[1]) / 2), f"e = {e:.2f} m", font(42, True), ACCENT, anchor="lm")
    label_box(d, (hp[0] + 18, hp[1] - 0.05 * k), "person centre", font(30), INK, anchor="lm")
    tsize = kind_val[1]
    label_box(d, (im.size[0] / 2, 60),
              f"{'B3-O target_size_m' if kind == 'b3o' else 'Static radius'} = {tsize:.2f} m   ->   "
              f"corridor-side extent e = {e:.2f} m", font(36, True), INK)
    label_box(d, (im.size[0] / 2, im.size[1] - 50),
              "blue: Comfort Space boundary (person centre -> AMR surface); floor disc: + AMR half width; "
              "outer band: + costmap buffer", font(24), INK)
    Image.alpha_composite(im, ov).convert("RGB").save(out)
    return {"e": round(e, 4), "target_or_radius": tsize, "kind": kind}


def robots(src: Path, out: Path):
    m = meta_of(src)
    im = Image.open(src).convert("RGBA")
    ov = Image.new("RGBA", im.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(ov)
    for name, (x, y) in m["robots"].items():
        p = px(m, x, y)
        label_box(d, (p[0], p[1] - 38), name, font(30, True), (255, 255, 255), bg=(0, 0, 0, 190))
    Image.alpha_composite(im, ov).convert("RGB").save(out)
    return {"robots": m["robots"]}


def trajectory(src: Path, out: Path, trial: Path, text: str = ""):
    """A recorded trial (its JSON) over an orthographic plan of the same scene:
    first global plan (blue), AMR 1 trajectory (black), final footprint."""
    m = meta_of(src)
    d0 = json.loads(Path(trial).read_text())
    row = d0["row"]
    from comfort_core.config import load_config
    cfg = load_config(common_path=row["config"])
    im = Image.open(src).convert("RGBA")
    ov = Image.new("RGBA", im.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(ov)
    plan = [px(m, x, y) for x, y in (d0.get("planned_path") or [])]
    if len(plan) > 1:
        d.line(plan, fill=BLUE + (255,), width=5)
    traj = [px(m, t[1], t[2]) for t in (d0.get("trajectory") or [])]
    if len(traj) > 1:
        d.line(traj, fill=(20, 20, 20, 255), width=4)
    pose = d0.get("final_pose")
    if pose:
        x, y, yaw = pose
        hl, hw = float(cfg["amr"]["footprint_length"]) / 2, float(cfg["amr"]["footprint_width"]) / 2
        c, s_ = math.cos(yaw), math.sin(yaw)
        fp = [px(m, x + u * c - v * s_, y + u * s_ + v * c) for u, v in ((hl, hw), (-hl, hw), (-hl, -hw), (hl, -hw))]
        ok = row["result"] == "SUCCESS"
        d.polygon(fp, fill=((0, 158, 115, 230) if ok else (213, 94, 0, 230)), outline=(0, 0, 0, 255))
        p0 = px(m, x, y)
        mech = row.get("deadlock_mechanism") or row["result"]
        label_box(d, (p0[0], p0[1] - 46), f"AMR 1: {'SUCCESS' if ok else mech}", font(28, True),
                  (255, 255, 255), bg=(0, 0, 0, 190))
    if text:
        label_box(d, (im.size[0] / 2, 40), text, font(30, True), INK)
    Image.alpha_composite(im, ov).convert("RGB").save(out)
    return {"trial": str(trial), "result": row["result"], "mechanism": row.get("deadlock_mechanism"),
            "scenario": row["scenario"], "config": row["config"]}


def banner(src: Path, out: Path, text: str, sub: str = ""):
    im = Image.open(src).convert("RGBA")
    ov = Image.new("RGBA", im.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(ov)
    w = im.size[0]
    f = font(max(28, w // 40), True)
    d.rectangle((0, 0, w, int(w / 40 * 1.9) + (int(w / 70 * 1.6) if sub else 0)), fill=(0, 0, 0, 235))
    d.text((24, 12), text, font=f, fill=(255, 255, 255))
    if sub:
        d.text((24, 12 + int(w / 40 * 1.35)), sub, font=font(max(18, w // 70)), fill=(230, 230, 230))
    Image.alpha_composite(im, ov).convert("RGB").save(out)
    return {"banner": text, "sub": sub}


def grid(out: Path, images, cols=2, titles=None, gap=12):
    ims = [Image.open(p).convert("RGB") for p in images]
    w, h = ims[0].size
    ims = [i if i.size == (w, h) else i.resize((w, h)) for i in ims]
    rows = math.ceil(len(ims) / cols)
    th = 64 if titles else 0
    canvas = Image.new("RGB", (cols * w + (cols - 1) * gap, rows * (h + th) + (rows - 1) * gap), "white")
    d = ImageDraw.Draw(canvas)
    for i, im in enumerate(ims):
        r, c = divmod(i, cols)
        x0, y0 = c * (w + gap), r * (h + th + gap)
        if titles:
            d.text((x0 + w / 2, y0 + th / 2), titles[i], font=font(40, True), fill=INK, anchor="mm")
        canvas.paste(im, (x0, y0 + th))
    canvas.save(out)
    return {"panels": [str(p) for p in images], "titles": titles}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("op", choices=("width", "extent", "robots", "banner", "grid", "trajectory"))
    ap.add_argument("--trial", default=None, help="trial JSON (op trajectory)")
    ap.add_argument("paths", nargs="+")
    ap.add_argument("--text", default="")
    ap.add_argument("--sub", default="")
    ap.add_argument("--cols", type=int, default=2)
    ap.add_argument("--titles", nargs="*", default=None)
    ap.add_argument("--x-at", type=float, default=-3.5)
    a = ap.parse_args()
    p = [Path(x) for x in a.paths]
    if a.op == "grid":
        r = grid(p[0], p[1:], a.cols, a.titles)
    elif a.op == "width":
        r = width(p[0], p[1], a.x_at)
    elif a.op == "extent":
        r = extent(p[0], p[1])
    elif a.op == "trajectory":
        r = trajectory(p[0], p[1], Path(a.trial), a.text)
    elif a.op == "robots":
        r = robots(p[0], p[1])
    else:
        r = banner(p[0], p[1], a.text, a.sub)
    print(json.dumps(r))
    return 0


if __name__ == "__main__":
    sys.exit(main())
