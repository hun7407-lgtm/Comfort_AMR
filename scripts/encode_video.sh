#!/usr/bin/env bash
# ---------------------------------------------------------------
# Encode recorded frames into an mp4.
#
#   ./scripts/encode_video.sh <frames_dir> <out.mp4> [fps]
#
# Uses a system ffmpeg when there is one, otherwise the ffmpeg that ships
# inside Isaac Sim's Python (imageio_ffmpeg). fps defaults to
# recording.fps in configs/common.yaml, which is the rate frames were taken
# at, so the video plays at simulated real time.
# ---------------------------------------------------------------
set -euo pipefail

PROJECT_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
FRAMES="${1:?frames directory}"
OUT="${2:?output mp4}"
FPS="${3:-$(python3 -c "import sys,yaml;print(yaml.safe_load(open('$PROJECT_ROOT/configs/common.yaml'))['recording']['fps'])")}"

if command -v ffmpeg >/dev/null 2>&1; then
    FFMPEG="$(command -v ffmpeg)"
else
    if [ -n "${ISAACSIM_PATH:-}" ] && [ -x "$ISAACSIM_PATH/python.sh" ]; then
        ISAACSIM_ROOT="$ISAACSIM_PATH"
    elif [ -x "$HOME/isaacsim/python.sh" ]; then
        ISAACSIM_ROOT="$HOME/isaacsim"
    else
        FOUND=$(find "$HOME" -maxdepth 4 -name isaac-sim.sh -type f 2>/dev/null | head -1)
        [ -n "$FOUND" ] || { echo "[ERROR] no ffmpeg and no Isaac Sim found" >&2; exit 1; }
        ISAACSIM_ROOT="$(dirname "$FOUND")"
    fi
    FFMPEG="$("$ISAACSIM_ROOT/python.sh" -c 'import imageio_ffmpeg; print(imageio_ffmpeg.get_ffmpeg_exe())' 2>/dev/null | tail -1)"
fi
[ -x "$FFMPEG" ] || { echo "[ERROR] ffmpeg not usable: $FFMPEG" >&2; exit 1; }

COUNT=$(find "$FRAMES" -name 'frame_*.jpg' | wc -l)
if [ "$COUNT" -eq 0 ]; then
    echo "[ERROR] no frames in $FRAMES" >&2
    exit 1
fi

# yuv420p and even dimensions so ordinary players open it.
"$FFMPEG" -hide_banner -loglevel error -y -framerate "$FPS" \
    -i "$FRAMES/frame_%05d.jpg" \
    -vf "scale=trunc(iw/2)*2:trunc(ih/2)*2" \
    -c:v libx264 -pix_fmt yuv420p -crf 20 "$OUT"
echo "[info] $COUNT frames at $FPS fps -> $OUT"
