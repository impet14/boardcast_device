#!/usr/bin/env bash
set -euo pipefail

# Configuration
SERVICE_NAME="boardcast_device"
SERVICE_FILE="boardcast_device.service"
INSTALL_DIR="/home/dev/boardcast_device"

# Ensure script runs as root
if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root" 
   exit 1
fi

# 1) Verify installation directory exists
if [[ ! -d "$INSTALL_DIR" ]]; then
    echo "Error: Installation directory $INSTALL_DIR not found"
    exit 1
fi

# 2) Copy systemd service file
echo "Installing systemd service..."
cp "$SERVICE_FILE" /etc/systemd/system/

# 3) Set permissions on .env file
if [[ -f "$INSTALL_DIR/.env" ]]; then
    echo "Securing configuration file..."
    chown dev:dev "$INSTALL_DIR/.env"
    chmod 600 "$INSTALL_DIR/.env"
fi

# 4) Enable and start service
echo "Starting service..."
systemctl daemon-reload
systemctl enable "$SERVICE_NAME"
systemctl restart "$SERVICE_NAME"

# 5) Verify installation
sleep 2
systemctl status "$SERVICE_NAME" --no-pager

echo -e "\n✅ Service installed successfully!"
echo "   Logs: journalctl -u $SERVICE_NAME -f"
echo "   Config: $INSTALL_DIR/.env"
echo "   Control: sudo systemctl [start|stop|restart] $SERVICE_NAME"