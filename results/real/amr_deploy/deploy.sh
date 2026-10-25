#!/bin/bash
# Put the whole AMR toolkit on a robot.   ./deploy.sh clabil@<host>
# Idempotent: safe to re-run.
set -e
HOST=${1:?usage: ./deploy.sh clabil@<host-or-ip>}
HERE="$(cd "$(dirname "$0")" && pwd)"

echo "--- copying scripts ---"
scp -q "$HERE"/bin/* "$HOST":/home/clabil/
ssh "$HOST" 'chmod +x ~/*.sh ~/*.py'

echo "--- foxglove_bridge ---"
if ! ssh "$HOST" 'test -d /opt/ros/humble/share/foxglove_bridge'; then
  scp -q "$HERE"/ros-humble-foxglove-bridge_*_arm64.deb "$HOST":/tmp/
  ssh "$HOST" 'echo 123 | sudo -S apt-get install -y ros-humble-rosx-introspection rapidjson-dev >/dev/null 2>&1; echo 123 | sudo -S dpkg -i /tmp/ros-humble-foxglove-bridge_*_arm64.deb'
else
  echo "  already installed"
fi

echo "--- done. next: ---"
echo "  ssh $HOST '~/start_all.sh'"
echo "  ssh $HOST '~/preflight.sh'      <-- run this BEFORE anything else"
