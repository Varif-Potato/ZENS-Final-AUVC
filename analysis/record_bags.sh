#!/usr/bin/env bash
# Record all ROS2 topics during a pool test
# Usage: ./record_bags.sh [output_name]

set -euo pipefail

NAME="${1:-pool_test_$(date +%Y%m%d_%H%M%S)}"
mkdir -p bags

echo "Recording to bags/$NAME ..."
ros2 bag record -o "bags/$NAME" -a
