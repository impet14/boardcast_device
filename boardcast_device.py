#!/usr/bin/env python3
"""
boardcast_device.py with GPIO fan control

- Reads BME280 via I2C (temp, humidity, pressure)
- Controls GPIO23 (pin 16) fan based on temperature
- Reads GPS NMEA from serial (/dev/ttyAMA0)
- Publishes combined JSON payload to MQTT
"""

import os, sys, time, json, logging
from datetime import datetime, timezone

import smbus2, bme280
import serial, pynmea2
import paho.mqtt.client as mqtt
from dotenv import load_dotenv

try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False
    logging.warning("RPi.GPIO not available - fan control disabled")

# ─── Configuration ──────────────────────────────────────────────────────────────
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
load_dotenv(os.path.join(BASE_DIR, ".env"))

# Environment variables with defaults
BROKER_HOST = os.getenv("BROKER_HOST")
BROKER_PORT = int(os.getenv("BROKER_PORT", "1883"))
DEVICE_ID = os.getenv("DEVICE_ID")
TOPIC_ROOT = os.getenv("TOPIC_ROOT", "/boardcast")
I2C_BUS = int(os.getenv("I2C_BUS", "1"))
I2C_ADDR = int(os.getenv("I2C_ADDR", "0x76"), 16)
GPS_PORT = os.getenv("GPS_PORT", "/dev/ttyAMA0")
GPS_BAUD = int(os.getenv("GPS_BAUD", "9600"))
PUBLISH_INTERVAL = int(os.getenv("PUBLISH_INTERVAL", "5"))

# GPIO Configuration
FAN_GPIO = 23
FAN_ON_TEMP = 35.0
FAN_OFF_TEMP = 32.0  # Hysteresis to prevent rapid cycling
FAN_STATE = False

# Validate critical settings
if not BROKER_HOST or not DEVICE_ID:
    sys.stderr.write("ERROR: BROKER_HOST and DEVICE_ID must be set in .env\n")
    sys.exit(1)

TOPIC = f"{TOPIC_ROOT}/{DEVICE_ID}"

# ─── Logging ────────────────────────────────────────────────────────────────────
LOG_LEVEL = os.getenv("LOG_LEVEL", "INFO").upper()
logging.basicConfig(
    level=getattr(logging, LOG_LEVEL, logging.INFO),
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%Y-%m-%dT%H:%M:%SZ"
)
logger = logging.getLogger()

# ─── Initialize Hardware ────────────────────────────────────────────────────────
def init_bme280():
    """Initialize BME280 sensor with retry logic"""
    for _ in range(3):
        try:
            bus = smbus2.SMBus(I2C_BUS)
            params = bme280.load_calibration_params(bus, I2C_ADDR)
            logger.info(f"BME280 initialized at 0x{I2C_ADDR:02X}")
            return bus, params
        except Exception as e:
            logger.error(f"BME280 init failed: {e}")
            time.sleep(2)
    logger.critical("BME280 initialization failed after 3 attempts")
    return None, None

def init_gps():
    """Initialize GPS serial connection with retry logic"""
    for _ in range(3):
        try:
            ser = serial.Serial(GPS_PORT, GPS_BAUD, timeout=1)
            logger.info(f"GPS initialized on {GPS_PORT}")
            return ser
        except Exception as e:
            logger.error(f"GPS init failed: {e}")
            time.sleep(2)
    logger.critical("GPS initialization failed after 3 attempts")
    return None

def init_gpio():
    """Initialize GPIO for fan control"""
    if not GPIO_AVAILABLE:
        return
    
    try:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(FAN_GPIO, GPIO.OUT)
        GPIO.output(FAN_GPIO, GPIO.LOW)
        logger.info(f"GPIO {FAN_GPIO} initialized for fan control")
        return True
    except Exception as e:
        logger.error(f"GPIO initialization failed: {e}")
        return False

# ─── Sensor Reading Functions ───────────────────────────────────────────────────
def read_bme(bus, params):
    try:
        d = bme280.sample(bus, I2C_ADDR, params)
        return {
            "temperature": round(d.temperature, 2),
            "humidity": round(d.humidity, 2),
            "pressure": round(d.pressure, 2),
        }
    except Exception as e:
        logger.error(f"BME280 read failed: {e}")
        return None

def read_gps(ser):
    try:
        deadline = time.time() + 1.0
        while time.time() < deadline:
            raw = ser.readline().decode(errors="ignore").strip()
            if raw.startswith(("$GPGGA", "$GPRMC")):
                try:
                    msg = pynmea2.parse(raw)
                    lat = getattr(msg, "latitude", None)
                    lon = getattr(msg, "longitude", None)
                    if lat is None or lon is None:
                        continue
                    return {
                        "latitude": lat,
                        "longitude": lon,
                        "altitude": getattr(msg, "altitude", None),
                        "num_sats": getattr(msg, "num_sats", None),
                    }
                except (pynmea2.ParseError, ValueError):
                    continue
        return None
    except Exception as e:
        logger.error(f"GPS read failed: {e}")
        return None

# ─── Fan Control ────────────────────────────────────────────────────────────────
def control_fan(temperature):
    """Control fan based on temperature with hysteresis"""
    global FAN_STATE
    
    if temperature is None or not GPIO_AVAILABLE:
        return
    
    try:
        if temperature > FAN_ON_TEMP and not FAN_STATE:
            GPIO.output(FAN_GPIO, GPIO.HIGH)
            FAN_STATE = True
            logger.info(f"Fan ON (Temp: {temperature}°C > {FAN_ON_TEMP}°C)")
        
        elif temperature < FAN_OFF_TEMP and FAN_STATE:
            GPIO.output(FAN_GPIO, GPIO.LOW)
            FAN_STATE = False
            logger.info(f"Fan OFF (Temp: {temperature}°C < {FAN_OFF_TEMP}°C)")
    except Exception as e:
        logger.error(f"Fan control failed: {e}")

# ─── MQTT Functions ─────────────────────────────────────────────────────────────
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        logger.info("MQTT connected")
    else:
        logger.error(f"MQTT connect failed ({rc})")

def on_disconnect(client, userdata, rc):
    logger.warning("MQTT disconnected, will reconnect")

def connect_mqtt(client):
    """Connect to MQTT broker with retry"""
    while True:
        try:
            client.connect(BROKER_HOST, BROKER_PORT, keepalive=60)
            return True
        except Exception as e:
            logger.error(f"MQTT connect failed: {e}, retrying in 10s")
            time.sleep(10)

# ─── Main Application ───────────────────────────────────────────────────────────
def main():
    global FAN_STATE
    
    # Initialize hardware
    i2c_bus, bme_params = init_bme280()
    gps_serial = init_gps()
    gpio_initialized = init_gpio()
    
    # Initialize MQTT client
    client = mqtt.Client(client_id=DEVICE_ID)
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    
    if not connect_mqtt(client):
        logger.critical("MQTT connection failed permanently")
        return
    
    client.loop_start()
    last_publish = 0
    
    try:
        while True:
            current_time = time.time()
            
            # Read sensors
            bme_data = read_bme(i2c_bus, bme_params) if i2c_bus and bme_params else None
            gps_data = read_gps(gps_serial) if gps_serial else None
            
            # Control fan based on temperature
            if bme_data:
                control_fan(bme_data["temperature"])
            
            # Publish at intervals
            if current_time - last_publish >= PUBLISH_INTERVAL:
                payload = {
                    "device_id": DEVICE_ID,
                    "timestamp": datetime.now(timezone.utc).isoformat(),
                    "fan_state": FAN_STATE,
                }
                
                if bme_data:
                    payload.update(bme_data)
                
                if gps_data:
                    payload["gps"] = gps_data
                
                try:
                    result = client.publish(TOPIC, json.dumps(payload), qos=1)
                    if result.rc == mqtt.MQTT_ERR_SUCCESS:
                        logger.info(f"Published to {TOPIC}")
                    else:
                        logger.error(f"Publish failed ({result.rc})")
                except Exception as e:
                    logger.error(f"MQTT publish error: {e}")
                
                last_publish = current_time
            
            # Sleep with shorter intervals for better responsiveness
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    except Exception as e:
        logger.critical(f"Fatal error: {e}", exc_info=True)
    finally:
        # Cleanup
        client.loop_stop()
        client.disconnect()
        
        if GPIO_AVAILABLE and gpio_initialized:
            try:
                GPIO.output(FAN_GPIO, GPIO.LOW)
                GPIO.cleanup()
                logger.info("GPIO cleaned up")
            except Exception:
                pass
        
        if gps_serial and gps_serial.is_open:
            try:
                gps_serial.close()
            except Exception:
                pass
        
        if i2c_bus:
            try:
                i2c_bus.close()
            except Exception:
                pass

if __name__ == "__main__":
    main()