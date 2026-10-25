#!/bin/bash
# Watch the control board's USB and whether it answers. Logs to /tmp/watch.log.
# Run with bringup DOWN so nothing else holds the port.
exec > /tmp/watch.log 2>&1
echo "=== watching from $(date +%H:%M:%S) ==="
( echo 123 | sudo -S dmesg -w 2>/dev/null | stdbuf -oL grep -iE "usb 1-2|cdc_acm|ch34" | while read -r l; do echo "[$(date +%H:%M:%S)] USB  $l"; done ) &
while true; do
  tgt=$(readlink /dev/ttySerial 2>/dev/null || echo NONE)
  out=$(timeout 3 python3 "$HOME/board_scan.py" "/dev/$tgt" 2>&1 | grep '\*\*\*' | head -1)
  if [ -n "$out" ]; then
    echo "[$(date +%H:%M:%S)] BOARD ANSWERED  ($tgt)  $out"
  else
    echo "[$(date +%H:%M:%S)] silent  (ttySerial -> $tgt)"
  fi
  sleep 2
done
