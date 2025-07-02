#!/usr/bin/env bash
set -euo pipefail

# 1) Update & install system packages
sudo apt update
sudo DEBIAN_FRONTEND=noninteractive apt install -y \
  python3-pip python3-venv python3-rpi.gpio i2c-tools

# 2) Enable I2C if not already on
sudo raspi-config nonint do_i2c 0

# 3) Create & activate virtualenv
python3 -m venv venv
source venv/bin/activate

# 4) Install Python deps
pip install --upgrade pip
pip install -r requirements.txt

# 5) Copy systemd unit & start service
sudo cp boardcast_device.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable boardcast_device
sudo systemctl start  boardcast_device

echo "✅ Installation complete. Check logs with: journalctl -u boardcast_device -f"
