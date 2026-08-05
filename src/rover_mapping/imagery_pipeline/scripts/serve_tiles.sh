#!/usr/bin/env bash
#
# serve_tiles.sh — Serve prepared offline tiles over HTTP for mapviz.
#
# Usage:
#   ./serve_tiles.sh [name] [port]     # default port 8000
#
# With no name, serves the whole imagery/ dir (all tile sets at once).
# Fully offline: this is just a static file server over local tiles.
#
# Binds 127.0.0.1 by default. If mapviz runs on a different machine (base
# station reading tiles off the rover, or vice versa), export BIND=0.0.0.0
# and use that host's IP in the mapviz base URL instead of localhost.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NAME="${1:-}"
PORT="${2:-8000}"
BIND="${BIND:-127.0.0.1}"
IMG_DIR="${REPO_ROOT}/imagery"

[[ -d "$IMG_DIR" ]] || { echo "ERROR: no imagery/ directory yet — run prepare_imagery.sh first"; exit 1; }

echo "Serving $IMG_DIR on ${BIND}:${PORT}"
echo
if [[ -n "$NAME" ]]; then
    echo "  mapviz Custom WMTS Base URL:"
    echo "    http://localhost:${PORT}/${NAME}/tiles/{level}/{x}/{y}.png"
else
    for d in "$IMG_DIR"/*/tiles; do
        [[ -d "$d" ]] || continue
        n=$(basename "$(dirname "$d")")
        echo "  http://localhost:${PORT}/${n}/tiles/{level}/{x}/{y}.png"
    done
fi
echo
echo "Ctrl-C to stop."
cd "$IMG_DIR"
exec python3 -m http.server "$PORT" --bind "$BIND"
