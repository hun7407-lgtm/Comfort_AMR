#!/bin/bash
# Open the point cloud in a browser.
#
# A local web server rather than file:// because Chrome refuses fetch() on
# file URLs, and the 12 MB of points are loaded with fetch.
cd "$(dirname "$0")"
PORT=${1:-8777}
python3 -m http.server "$PORT" >/dev/null 2>&1 &
SRV=$!
sleep 1
echo "serving on http://localhost:$PORT  (Ctrl+C to stop)"
xdg-open "http://localhost:$PORT/" >/dev/null 2>&1
trap "kill $SRV 2>/dev/null" EXIT
wait $SRV
