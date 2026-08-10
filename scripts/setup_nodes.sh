#!/usr/bin/env bash
set -e

# Helper: prepare package to run nodes with rosrun/roslaunch
# Usage: from repository root: bash scripts/setup_nodes.sh

SCRIPTS=(action_t.py qr_decoder.py ring_detector.py tt.py vision_bridge.py)

mkdir -p scripts || true

for f in "${SCRIPTS[@]}"; do
  if [ -f "$f" ]; then
    cp "$f" "scripts/"
    chmod +x "scripts/$f"
    echo "Copied and chmod +x scripts/$f"
  else
    echo "Warning: $f not found in repo root"
  fi
done

echo "Done. Now put this repository into a catkin workspace (src/), run catkin_make and source devel/setup.bash." 
