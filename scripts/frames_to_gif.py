#!/usr/bin/env python3
"""Turn a directory of frame_*.jpg into an animated GIF.

    python3 scripts/frames_to_gif.py <frames_dir> <out.gif> [--fps 5] [--width 900]

GIFs of a whole trial get large quickly, so frames are scaled down and
share one palette. The mp4 from scripts/encode_video.sh is the better
file for watching; the GIF is for pasting into documents.
"""

import argparse
import sys
from pathlib import Path

from PIL import Image


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("frames")
    parser.add_argument("out")
    parser.add_argument("--fps", type=float, default=5.0)
    parser.add_argument("--width", type=int, default=900)
    args = parser.parse_args()

    paths = sorted(Path(args.frames).glob("frame_*.jpg"))
    if not paths:
        print(f"[ERROR] no frames in {args.frames}", file=sys.stderr)
        return 1

    frames = []
    for p in paths:
        img = Image.open(p).convert("RGB")
        if img.width > args.width:
            img = img.resize((args.width, round(img.height * args.width / img.width)),
                             Image.LANCZOS)
        frames.append(img)

    palette = frames[len(frames) // 2].quantize(colors=128, method=Image.MEDIANCUT)
    quantized = [f.quantize(palette=palette, dither=Image.Dither.NONE) for f in frames]
    quantized[0].save(args.out, save_all=True, append_images=quantized[1:],
                      duration=int(1000 / args.fps), loop=0, optimize=True)
    print(f"[info] {len(frames)} frames at {args.fps:g} fps -> {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
