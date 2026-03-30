#!/usr/bin/env bash
set -euo pipefail

python3 -m pip install --user --break-system-packages -r "$(dirname "$0")/../requirements-web.txt"
echo "The detector runs on the host via yolo_api.py; no model is needed in the VM."
