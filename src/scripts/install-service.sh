#!/usr/bin/env bash
# Run this once on the Pi to install and enable the LUCIA web server as a systemd service.
# After this, the web panel starts automatically on every boot.
#
# Usage:
#   cd ~/repos/group-project-group-37-project-l-u-c-i-a/src/scripts
#   bash install-service.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVICE_SRC="$SCRIPT_DIR/lucia.service"
SERVICE_DST="/etc/systemd/system/lucia.service"

echo "=== LUCIA service installer ==="
echo

# Confirm deps
if ! python3 -c "import fastapi, uvicorn" &>/dev/null; then
    echo "Installing Python dependencies..."
    pip3 install fastapi "uvicorn[standard]" --break-system-packages
fi

echo "Copying service file → $SERVICE_DST"
sudo cp "$SERVICE_SRC" "$SERVICE_DST"

echo "Enabling and starting lucia service..."
sudo systemctl daemon-reload
sudo systemctl enable lucia
sudo systemctl start lucia

echo
echo "=== Done ==="
echo
sudo systemctl status lucia --no-pager -l
echo
echo "Web panel → http://10.42.0.1:8000"
echo
echo "Useful commands:"
echo "  sudo systemctl status lucia    — check if running"
echo "  sudo systemctl restart lucia   — restart after code changes"
echo "  sudo systemctl stop lucia      — stop it"
echo "  journalctl -u lucia -f         — live logs"
