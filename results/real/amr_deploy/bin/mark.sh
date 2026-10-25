#!/bin/bash
# Timestamp a phase boundary into the bag's companion file. ./mark.sh "<label>"
BAG=$(cat /tmp/current_bag 2>/dev/null)
printf '%s  %s\n' "$(date +%s.%N)" "$*" >> "${BAG}_marks.txt"
printf '%s  %s\n' "$(date +%H:%M:%S)" "$*"
