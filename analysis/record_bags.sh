#!/usr/bin/env bash
# Record all ROS2 topics during a pool test
# Usage: ./record_bags.sh [output_name]
#
# Records ALL topics in MCAP format with zstd compression.
# Output is stored in bag_files/<name>/.

set -euo pipefail

NAME="${1:-pool_test_$(date +%Y%m%d_%H%M%S)}"
OUTDIR="$(cd "$(dirname "$0")/.." && pwd)/analysis/bag_files"
mkdir -p "$OUTDIR"

# Warn if disk space is low (below 500 MB)
available_kb=$(df --output=avail "$OUTDIR" 2>/dev/null | tail -1 || echo 0)
available_mb=$((available_kb / 1024))
if [ "$available_mb" -lt 500 ]; then
    echo "WARNING: Low disk space — only ${available_mb} MB available at $OUTDIR"
    echo "Consider freeing space or recording fewer topics."
    echo ""
fi

echo "============================================"
echo " Recording ALL topics to: $OUTDIR/$NAME/"
echo " Format:  MCAP"
echo " Compression: zstd (file-level)"
echo "============================================"
echo ""

ros2 bag record \
    -a \
    -s mcap \
    --compression-mode file \
    --compression-format zstd \
    --compression-queue-size 50 \
    -o "$OUTDIR/$NAME"
