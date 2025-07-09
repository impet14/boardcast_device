#!/usr/bin/env python3
"""
Send configuration update or command to boardcast_device via MQTT config topic.

Usage:
    python3 send_config.py [--env .env] [--on-temp 30] [--off-temp 28] [--command <cmd>]

Examples:
    python3 send_config.py --on-temp 30 --off-temp 28
    python3 send_config.py --command reboot
"""

import os
import sys
import json
import argparse
from dotenv import load_dotenv
import paho.mqtt.client as mqtt

def main():
    parser = argparse.ArgumentParser(description="Send config/command to boardcast_device")
    parser.add_argument("--env", default=".env", help="Path to .env file (default: .env)")
    parser.add_argument("--on-temp", type=float, help="Fan ON temperature")
    parser.add_argument("--off-temp", type=float, help="Fan OFF temperature")
    parser.add_argument("--command", type=str, help="Send a command (e.g., reboot)")
    args = parser.parse_args()

    # Load environment variables
    load_dotenv(args.env)
    BROKER_HOST = os.getenv("BROKER_HOST")
    BROKER_PORT = int(os.getenv("BROKER_PORT", "1883"))
    DEVICE_ID = os.getenv("DEVICE_ID")
    TOPIC_ROOT = os.getenv("TOPIC_ROOT", "/boardcast")

    if not BROKER_HOST or not DEVICE_ID:
        print("BROKER_HOST and DEVICE_ID must be set in .env or via environment.", file=sys.stderr)
        sys.exit(1)

    topic = f"{TOPIC_ROOT}/{DEVICE_ID}/config"
    payload = {}

    if args.on_temp is not None:
        payload["FAN_ON_TEMP"] = args.on_temp
    if args.off_temp is not None:
        payload["FAN_OFF_TEMP"] = args.off_temp
    if args.command:
        payload["COMMAND"] = args.command

    if not payload:
        print("Nothing to send. Specify --on-temp, --off-temp, or --command.", file=sys.stderr)
        sys.exit(1)

    client = mqtt.Client()
    try:
        client.connect(BROKER_HOST, BROKER_PORT, keepalive=10)
        client.loop_start()
        result = client.publish(topic, json.dumps(payload), qos=1)
        result.wait_for_publish()
        print(f"Published to {topic}: {payload}")
        client.loop_stop()
        client.disconnect()
    except Exception as e:
        print(f"Failed to send config/command: {e}", file=sys.stderr)
        sys.exit(1)

if __name__ == "__main__":
    main()