#!/usr/bin/env python3
"""
boardcast_device.py

- Reads BME280 via I2C (temp, humidity, pressure).
- Reads GPS NMEA from serial (/dev/ttyAMA0).
- Publishes a combined JSON payload to MQTT under:
    {TOPIC_ROOT}/{DEVICE_ID}

- Optionally: add subscriber callbacks in this same file.
"""

import os, sys, time, json, logging
from datetime import datetime, timezone

import smbus2, bme280
import serial, pynmea2
import paho.mqtt.client as mqtt
from dotenv import load_dotenv

# ─── Load configuration from .env ───────────────────────────────────────────────
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
load_dotenv(os.path.join(BASE_DIR, ".env"))

BROKER_HOST      = os.getenv("BROKER_HOST")
BROKER_PORT      = int(os.getenv("BROKER_PORT", "1883"))
DEVICE_ID        = os.getenv("DEVICE_ID")
TOPIC_ROOT       = os.getenv("TOPIC_ROOT", "/boardcast")
I2C_BUS          = int(os.getenv("I2C_BUS", "1"))
I2C_ADDR         = int(os.getenv("I2C_ADDR", "0x76"), 16)
GPS_PORT         = os.getenv("GPS_PORT", "/dev/ttyAMA0")
GPS_BAUD         = int(os.getenv("GPS_BAUD", "9600"))
GPS_TIMEOUT      = float(os.getenv("GPS_TIMEOUT", "1"))
PUBLISH_INTERVAL = int(os.getenv("PUBLISH_INTERVAL", "5"))

# Validate critical settings
if not BROKER_HOST or not DEVICE_ID:
    sys.stderr.write("ERROR: BROKER_HOST and DEVICE_ID must be set in .env\n")
    sys.exit(1)

TOPIC = f"{TOPIC_ROOT}/{DEVICE_ID}"

# ─── Logging ─────────────────────────────────────────────────────────────────────
LOG_LEVEL = os.getenv("LOG_LEVEL", "INFO").upper()
logging.basicConfig(
    level=getattr(logging, LOG_LEVEL, logging.INFO),
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%Y-%m-%dT%H:%M:%SZ"
)
logger = logging.getLogger()

# ─── Initialize BME280 via smbus2 + bme280 ───────────────────────────────────────
try:
    i2c_bus = smbus2.SMBus(I2C_BUS)
    bme_params = bme280.load_calibration_params(i2c_bus, I2C_ADDR)
except Exception as e:
    logger.error(f"BME280 init failed at 0x{I2C_ADDR:02X}: {e}")
    sys.exit(1)

def read_bme():
    d = bme280.sample(i2c_bus, I2C_ADDR, bme_params)
    return {
        "temperature": round(d.temperature, 2),
        "humidity":    round(d.humidity,    2),
        "pressure":    round(d.pressure,    2),
    }

# ─── Initialize GPS serial link ────────────────────────────────────────────────
try:
    gps_serial = serial.Serial(GPS_PORT, GPS_BAUD, timeout=GPS_TIMEOUT)
except Exception as e:
    logger.error(f"GPS init failed on {GPS_PORT}: {e}")
    sys.exit(1)

def read_gps():
    deadline = time.time() + GPS_TIMEOUT
    while time.time() < deadline:
        raw = gps_serial.readline().decode(errors="ignore").strip()
        if raw.startswith(("$GPGGA", "$GPRMC")):
            try:
                msg = pynmea2.parse(raw)
                # Validate latitude and longitude
                lat = getattr(msg, "latitude", None)
                lon = getattr(msg, "longitude", None)
                # Sometimes pynmea2 returns '' instead of None
                if not lat or not lon:
                    raise ValueError("Empty latitude or longitude")
                # Check for obviously invalid values (e.g., not a float)
                float(lat)
                float(lon)
                return {
                    "latitude":  lat,
                    "longitude": lon,
                    "altitude":  getattr(msg, "altitude", None),
                    "num_sats":  getattr(msg, "num_sats", None),
                }
            except (pynmea2.ParseError, ValueError) as e:
                logger.warning(f"Invalid GPS data: {raw} ({e})")
                continue
    # If no valid data found
    return {"latitude": None, "longitude": None, "altitude": None, "num_sats": None}

# ─── MQTT callbacks ────────────────────────────────────────────────────────────
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        logger.info("MQTT connected")
    else:
        logger.error(f"MQTT connect failed ({rc})")

def on_disconnect(client, userdata, rc):
    logger.warning("MQTT disconnected, will reconnect")

def safe_read_bme():
    try:
        return read_bme()
    except Exception as e:
        logger.error(f"BME280 read failed: {e}")
        return {"temperature": None, "humidity": None, "pressure": None}

def safe_read_gps():
    try:
        return read_gps()
    except Exception as e:
        logger.error(f"GPS read failed: {e}")
        return {"latitude": None, "longitude": None, "altitude": None, "num_sats": None}

# ─── Main loop ──────────────────────────────────────────────────────────────────
def main():
    global i2c_bus, bme_params, gps_serial
    client = mqtt.Client(client_id=DEVICE_ID)
    client.on_connect    = on_connect
    client.on_disconnect = on_disconnect

    while True:
        try:
            client.connect(BROKER_HOST, BROKER_PORT, keepalive=60)
            client.loop_start()
            break
        except Exception as e:
            logger.error(f"MQTT connect failed: {e}, retrying in 10s")
            time.sleep(10)

    try:
        while True:
            # Self-heal BME280 if needed
            if i2c_bus is None or bme_params is None:
                try:
                    i2c_bus = smbus2.SMBus(I2C_BUS)
                    bme_params = bme280.load_calibration_params(i2c_bus, I2C_ADDR)
                    logger.info("BME280 re-initialized")
                except Exception as e:
                    logger.error(f"BME280 re-init failed: {e}")

            # Self-heal GPS if needed
            if gps_serial is None or not gps_serial.is_open:
                try:
                    gps_serial = serial.Serial(GPS_PORT, GPS_BAUD, timeout=GPS_TIMEOUT)
                    logger.info("GPS serial re-initialized")
                except Exception as e:
                    logger.error(f"GPS serial re-init failed: {e}")

            payload = {
                "device_id": DEVICE_ID,
                "timestamp": datetime.now(timezone.utc).isoformat(),
                **safe_read_bme(),
                "gps": safe_read_gps()
            }
            try:
                result = client.publish(TOPIC, json.dumps(payload), qos=1)
                if result.rc == mqtt.MQTT_ERR_SUCCESS:
                    logger.info(f"Published to {TOPIC}")
                else:
                    logger.error(f"Publish failed ({result.rc})")
            except Exception as e:
                logger.error(f"MQTT publish error: {e}")

            time.sleep(PUBLISH_INTERVAL)
    except KeyboardInterrupt:
        logger.info("Interrupted by user, exiting")
    except Exception as e:
        logger.critical(f"Fatal error in main loop: {e}", exc_info=True)
    finally:
        try:
            client.loop_stop()
            client.disconnect()
        except Exception:
            pass
        try:
            if gps_serial and gps_serial.is_open:
                gps_serial.close()
        except Exception:
            pass
        try:
            if i2c_bus:
                i2c_bus.close()
        except Exception:
            pass

if __name__ == "__main__":
    main()
