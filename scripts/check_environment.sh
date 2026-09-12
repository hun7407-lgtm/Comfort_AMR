#!/usr/bin/env bash
# ---------------------------------------------------------------
# Comfort_AMR - environment check
#
# Verifies every prerequisite needed to run the B2/B3 experiments.
# Read-only: this script never installs or modifies anything.
#
#   Usage:  ./scripts/check_environment.sh
#
# Exit code 0 = all required checks passed.
# ---------------------------------------------------------------

# Deliberately NOT using `set -e`: we want every check to run even
# when an earlier one fails, so the user sees the full picture.
set -uo pipefail

PROJECT_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"

RED=$'\e[31m'; GRN=$'\e[32m'; YEL=$'\e[33m'; BLU=$'\e[34m'; RST=$'\e[0m'

FAIL_COUNT=0
WARN_COUNT=0

section() { printf '\n%s=== %s ===%s\n' "$BLU" "$1" "$RST"; }
ok()      { printf '  %s[ OK ]%s   %s\n'   "$GRN" "$RST" "$1"; }
warn()    { printf '  %s[WARN]%s   %s\n'   "$YEL" "$RST" "$1"; WARN_COUNT=$((WARN_COUNT+1)); }
fail()    { printf '  %s[FAIL]%s   %s\n'   "$RED" "$RST" "$1"; FAIL_COUNT=$((FAIL_COUNT+1)); }
info()    { printf '           %s\n' "$1"; }

# ---------------------------------------------------------------
section "Operating system"
# ---------------------------------------------------------------
if [ -r /etc/os-release ]; then
    . /etc/os-release
    info "$PRETTY_NAME  (kernel $(uname -r))"
    if [ "${VERSION_ID:-}" = "24.04" ]; then
        ok "Ubuntu 24.04 - matches the ROS 2 Jazzy target"
    else
        warn "Expected Ubuntu 24.04; ROS 2 distro choice may differ"
    fi
else
    fail "/etc/os-release not readable - cannot identify the OS"
fi

# ---------------------------------------------------------------
section "GPU and NVIDIA driver"
# ---------------------------------------------------------------
if command -v nvidia-smi >/dev/null 2>&1; then
    GPU_NAME=$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1)
    DRV_VER=$(nvidia-smi --query-gpu=driver_version --format=csv,noheader 2>/dev/null | head -1)
    VRAM_MB=$(nvidia-smi --query-gpu=memory.total --format=csv,noheader,nounits 2>/dev/null | head -1)
    if [ -n "$GPU_NAME" ]; then
        ok "$GPU_NAME  |  driver $DRV_VER  |  ${VRAM_MB} MiB VRAM"
        # Isaac Sim 5.1 requires driver 535+; RTX 50-series needs 570+.
        DRV_MAJOR=${DRV_VER%%.*}
        if [ "${DRV_MAJOR:-0}" -lt 535 ] 2>/dev/null; then
            fail "Driver $DRV_VER is below the 535 minimum for Isaac Sim 5.1"
        fi
        if [ "${VRAM_MB:-0}" -lt 8000 ] 2>/dev/null; then
            warn "Less than 8 GB VRAM - Isaac Sim may struggle"
        fi
    else
        fail "nvidia-smi ran but reported no GPU"
    fi
else
    fail "nvidia-smi not found - no NVIDIA driver installed"
fi

if command -v vulkaninfo >/dev/null 2>&1; then
    ok "vulkaninfo present (useful when debugging renderer failures)"
else
    warn "vulkaninfo not found - install with: sudo apt install vulkan-tools"
fi

# ---------------------------------------------------------------
section "Isaac Sim"
# ---------------------------------------------------------------
# Never hardcode the install path. Honour an explicit override first,
# then try the conventional location, then search the home directory.
find_isaacsim() {
    if [ -n "${ISAACSIM_PATH:-}" ] && [ -x "$ISAACSIM_PATH/isaac-sim.sh" ]; then
        echo "$ISAACSIM_PATH"; return 0
    fi
    for guess in "$HOME/isaacsim" "$HOME/.local/share/ov/pkg"/isaac*sim*; do
        [ -x "$guess/isaac-sim.sh" ] && { echo "$guess"; return 0; }
    done
    local found
    found=$(find "$HOME" -maxdepth 4 -name isaac-sim.sh -type f 2>/dev/null | head -1)
    [ -n "$found" ] && { dirname "$found"; return 0; }
    return 1
}

if ISAACSIM_ROOT=$(find_isaacsim); then
    ok "Found at $ISAACSIM_ROOT"
    if [ -r "$ISAACSIM_ROOT/VERSION" ]; then
        info "version $(cat "$ISAACSIM_ROOT/VERSION")"
    fi

    if [ -x "$ISAACSIM_ROOT/python.sh" ]; then
        PY_VER=$("$ISAACSIM_ROOT/python.sh" -c 'import sys; print("%d.%d.%d" % sys.version_info[:3])' 2>/dev/null | tail -1)
        ok "bundled Python $PY_VER"
        info "This differs from the system Python on purpose."
        info "Never import rclpy from it; ROS 2 nodes run as separate processes."
    else
        fail "python.sh missing - the Isaac Sim install looks incomplete"
    fi

    BRIDGE="$ISAACSIM_ROOT/exts/isaacsim.ros2.bridge"
    if [ -d "$BRIDGE" ]; then
        ok "ROS 2 bridge extension present"
        for distro in jazzy humble; do
            if [ -d "$BRIDGE/$distro/lib" ]; then
                info "bundled $distro libraries: $(ls "$BRIDGE/$distro/lib" | wc -l) files"
            fi
        done
    else
        fail "isaacsim.ros2.bridge not found - the ROS 2 bridge is unavailable"
    fi
else
    fail "Isaac Sim not found. Set ISAACSIM_PATH to the install directory."
fi

# ---------------------------------------------------------------
section "ROS 2"
# ---------------------------------------------------------------
if [ -d /opt/ros ]; then
    info "installed distros: $(ls /opt/ros | tr '\n' ' ')"
fi

if [ -z "${ROS_DISTRO:-}" ] && [ -f /opt/ros/jazzy/setup.bash ]; then
    # Source it just for this check so the script is usable from a
    # plain shell without the user having sourced anything first.
    # shellcheck disable=SC1091
    . /opt/ros/jazzy/setup.bash
    info "sourced /opt/ros/jazzy/setup.bash for this check"
fi

if command -v ros2 >/dev/null 2>&1; then
    ok "ros2 CLI at $(command -v ros2)"
    if [ "${ROS_DISTRO:-}" = "jazzy" ]; then
        ok "ROS_DISTRO=jazzy"
    else
        warn "ROS_DISTRO='${ROS_DISTRO:-<unset>}' - this project targets jazzy"
    fi
    info "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<unset, defaults to rmw_fastrtps_cpp>}"
    info "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset, defaults to 0>}"
else
    fail "ros2 not found - see docs/SETUP.md for the install steps"
fi

for tool in colcon rosdep; do
    if command -v "$tool" >/dev/null 2>&1; then
        ok "$tool at $(command -v "$tool")"
    else
        fail "$tool not found - see docs/SETUP.md"
    fi
done

# ---------------------------------------------------------------
section "Project"
# ---------------------------------------------------------------
info "root: $PROJECT_ROOT"

if command -v python3 >/dev/null 2>&1; then
    ok "system Python $(python3 --version 2>&1 | awk '{print $2}')"
else
    fail "python3 not found"
fi

if python3 -c 'import yaml' 2>/dev/null; then
    ok "PyYAML available to the system Python"
else
    fail "PyYAML missing - install with: sudo apt install python3-yaml"
fi

if [ -n "${ISAACSIM_ROOT:-}" ] && [ -x "$ISAACSIM_ROOT/python.sh" ]; then
    if "$ISAACSIM_ROOT/python.sh" -c 'import yaml' >/dev/null 2>&1; then
        ok "PyYAML available to the Isaac Sim Python"
    else
        warn "PyYAML missing in Isaac Sim Python; config loading there will fail"
        info "install with: $ISAACSIM_ROOT/python.sh -m pip install pyyaml"
    fi
fi

# AMR asset presence. Absent is expected until STEP 3.
AMR_USD_COUNT=$(find "$PROJECT_ROOT/assets/amr" -name '*.usd*' 2>/dev/null | wc -l)
if [ "$AMR_USD_COUNT" -gt 0 ]; then
    ok "assets/amr holds $AMR_USD_COUNT USD file(s)"
    find "$PROJECT_ROOT/assets/amr" -name '*.usd*' -printf '           %s bytes  %p\n' 2>/dev/null
else
    warn "assets/amr is empty - the AMR model has not been imported yet (STEP 3)"
fi

# ---------------------------------------------------------------
section "Git"
# ---------------------------------------------------------------
if git -C "$PROJECT_ROOT" rev-parse --git-dir >/dev/null 2>&1; then
    ok "git repository"
    info "branch:  $(git -C "$PROJECT_ROOT" branch --show-current 2>/dev/null)"
    info "remote:  $(git -C "$PROJECT_ROOT" remote get-url origin 2>/dev/null || echo '<none>')"

    G_NAME=$(git -C "$PROJECT_ROOT" config user.name  || true)
    G_MAIL=$(git -C "$PROJECT_ROOT" config user.email || true)
    if [ -n "$G_NAME" ] && [ -n "$G_MAIL" ]; then
        ok "commit identity: $G_NAME <$G_MAIL>"
    else
        fail "git user.name / user.email not set - commits will be rejected"
    fi

    DIRTY=$(git -C "$PROJECT_ROOT" status --porcelain | wc -l)
    info "uncommitted changes: $DIRTY file(s)"
else
    fail "$PROJECT_ROOT is not a git repository"
fi

# ---------------------------------------------------------------
section "Summary"
# ---------------------------------------------------------------
printf '  failures: %d   warnings: %d\n\n' "$FAIL_COUNT" "$WARN_COUNT"
if [ "$FAIL_COUNT" -eq 0 ]; then
    printf '  %sEnvironment is ready.%s\n\n' "$GRN" "$RST"
    exit 0
else
    printf '  %sFix the failures above before continuing.%s\n\n' "$RED" "$RST"
    exit 1
fi
