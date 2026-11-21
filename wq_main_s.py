#pip install pymodbus
#pip install gpiozero
#pip install pyserial
#pip install smtplib
#pip install sqlite3

import logging
import math
import os
import smtplib
import sqlite3
import struct

#import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import UTC, datetime
from email.message import EmailMessage

import serial

#from xmlrpc import server
from gpiozero import Button, GPIOZeroError, OutputDevice
from pymodbus.client import ModbusTcpClient
from pymodbus.exceptions import ModbusException

# ==============================================================================
# Use this when testing on a non-RPi platform to avoid GPIO errors:
# ==============================================================================
#os.environ["GPIOZERO_PIN_FACTORY"] = "mock"

# ==============================================================================
# 1. STORAGE & CYCLE TIMING CONFIGURATION
# ==============================================================================
MOUNT_POINT = '/mnt/sensor_ssd'
FALLBACK_DIR = "/var/lib/station_data"  # Local SD card fallback directory
DB_FILENAME = "all_sensors_data.db"
#DB_PATH = os.path.join(MOUNT_POINT, 'all_sensors_data.db')
LOG_INTERVAL_SECONDS = 900  # 15-minute (900s) measurement cycle

# ==============================================================================
# 2. GPIO RELAYS, HIGH-LEVEL FLOAT SWITCH & TANK THRESHOLDS
# ==============================================================================
PUMP_GPIO_PIN = 17          
# Digital Loggers IoT Power Relay II (120V AC Pump)
DRAIN_VALVE_GPIO_PIN = 27    # 12V Opto-Isolated Relay -> 1" Motorized Ball Valve
HEATER_GPIO_PIN = 22         # Dedicated IoT Power Relay II (120V AC Heater)
AC_GPIO_PIN = 23             # Dedicated Relay/Contactor (Air Conditioner)
HIGH_FLOAT_GPIO_PIN = 24     # Tank High-Level Float Switch (NC to GND; opens when high)
RINSE_VALVE_GPIO_PIN = 25    # 12V Opto-Isolated Relay -> Tank Auto-Rinse / Backflush Valve

pump_relay = OutputDevice(PUMP_GPIO_PIN, active_high=True, initial_value=False)
drain_valve = OutputDevice(DRAIN_VALVE_GPIO_PIN, active_high=True, initial_value=False)
heater_relay = OutputDevice(HEATER_GPIO_PIN, active_high=True, initial_value=False)
ac_relay = OutputDevice(AC_GPIO_PIN, active_high=True, initial_value=False)
rinse_valve = OutputDevice(RINSE_VALVE_GPIO_PIN, active_high=True, initial_value=False)

# Normally Closed float switch wired between GPIO 24 and GND:
high_float_switch = Button(HIGH_FLOAT_GPIO_PIN, pull_up=True, bounce_time=0.2)

TARGET_FILL_LEVEL = 1.50     # Stop pumping when CS451 tank level reaches this depth (ft)
MAX_FILL_SECONDS = 180       # Safety cutoff so pump never runs dry indefinitely
RINSE_DURATION_SECONDS = 15  # Auto-rinse / backflush duration at start of drain
DRAIN_DURATION_SECONDS = 60  # Must be >= 60s to charge 2-wire valve return capacitor

# Dry-Run / Stall Detection Settings
DRY_RUN_CHECK_INTERVAL = 15      # Check level gain every 15 seconds while pumping
MIN_LEVEL_GAIN_PER_CHECK = 0.02  # Minimum ft rise expected every 15 seconds

# ==============================================================================
# 3. TRAILER CLIMATE & ISCO 6712 SAMPLING THRESHOLDS
# ==============================================================================
TRAILER_TEMP_SENSOR_ID = "trailer_temp_1"
HEATER_ON_TEMP_C = (65.0 - 32.0) * 5.0 / 9.0    # 18.33 °C (65°F)
HEATER_OFF_TEMP_C = (68.0 - 32.0) * 5.0 / 9.0   # 20.00 °C (68°F)
AC_ON_TEMP_C = (80.0 - 32.0) * 5.0 / 9.0        # 26.67 °C (80°F)
AC_OFF_TEMP_C = (76.0 - 32.0) * 5.0 / 9.0       # 24.44 °C (76°F)
AC_MIN_OFF_SECONDS = 300                        # 5-minute compressor short-cycle protection

MIN_VALID_TEMP_C = -40.0
MAX_VALID_TEMP_C = 70.0
FREEZE_ALERT_TEMP_C = 2.0
ALERT_COOLDOWN_SEC = 60 * 60
ISCO_BOTTLE_WARN_THRESHOLD = 22  # Warn when bottle count reaches 22 of 24

# ISCO_01: Every 90 minutes when stream water level > 0.01 ft
ISCO_01_LEVEL_THRESHOLD = 0.01
ISCO_01_COOLDOWN_SEC = 90 * 60

# ISCO_02: Both stream water level AND turbidity exceed thresholds (15-min cooldown)
ISCO_02_LEVEL_THRESHOLD = 0.50       # <-- ADJUST: Stream water level threshold (ft)
ISCO_02_TURBIDITY_THRESHOLD = 100.0  # <-- ADJUST: Hach Solitax turbidity threshold (NTU/FNU)
ISCO_02_COOLDOWN_SEC = 15 * 60

# Optional Email Alert Settings
SMTP_SERVER = ""
SMTP_PORT = 587
SMTP_USER = "your-alert-email@example.com"
SMTP_PASS = "your-app-password"
ALERT_RECIPIENT = "recipient@example.com"

# ==============================================================================
# 4. DEVICE PORTS & NETWORK ADDRESSES
# ==============================================================================
# CS655_SENSORS = {
#     "cs655_01": "/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0-port0",
#     "cs655_02": "/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.2:1.0-port0",
# }
#Update with actual serial port path for the SDI12 sensor
SDI12_PORT = "/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0-port0"
#update with actual SDI-12 address of the CS451 tank level sensor
CS451_SDI12_ADDR = "0"
#Update with the actual SDI-12 addresses
SDI12_SENSORS = {
    "5tm_01": {"addr": "1", "type": "5tm"},
    "5tm_02": {"addr": "2", "type": "5tm"},
    "ysi_bubbler": {"addr": "3", "type": "bubbler"},
}
# Update with actual IP address of SC4500 on your network; default Modbus TCP port is 502
SC4500_IP = "192.168.50.10"
SC4500_PORT = 502
# Example Register Addresses & Sensor Unit IDs (Update to match your SC4500 map)
SOLITAX_UNIT_ID = 1
SOLITAX_TURBIDITY_REG = 0
SOLITAX_SOLIDS_REG = 2
NITRATAX_UNIT_ID = 2
NITRATAX_NITRATE_REG = 0
# Update with actual serial port paths for the ISCO 6712 samplers
ISCO_SAMPLERS = {
    "isco_01": "/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.6:1.0-port0",
    "isco_02": "/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.7:1.0-port0",
}


# ==============================================================================
# DATABASE INITIALIZATION & LOGGING HELPERS
# ==============================================================================
def init_db():
    using_fallback = False
    
    if os.path.ismount(MOUNT_POINT):
        active_dir = MOUNT_POINT
    else:
        using_fallback = True
        active_dir = FALLBACK_DIR
        os.makedirs(active_dir, exist_ok=True)
        logger.warning(
            f"SSD not mounted at {MOUNT_POINT}. Falling back to local SD card: {active_dir}"
        )

    db_path = os.path.join(active_dir, DB_FILENAME)
    conn = sqlite3.connect(db_path, timeout=30)
    cursor = conn.cursor()
    cursor.execute("PRAGMA journal_mode=WAL;")
    cursor.execute("PRAGMA synchronous=NORMAL;")

    cursor.execute('''
        CREATE TABLE IF NOT EXISTS soil_readings (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp_utc TEXT NOT NULL,
            sensor_id TEXT NOT NULL,
            vwc REAL,
            ec REAL,
            temperature_c REAL,
            water_level REAL,
            turbidity REAL,
            suspended_solids REAL,
            nitrate_mg_l REAL,
            isco_bottle INTEGER,
            isco_status TEXT,
            synced INTEGER DEFAULT 0
        )
    ''')
    cursor.execute('CREATE INDEX IF NOT EXISTS idx_synced ON soil_readings (synced);')
    cursor.execute('CREATE INDEX IF NOT EXISTS idx_sensor_time ON soil_readings (sensor_id, timestamp_utc);')

    cursor.execute('''
        CREATE TABLE IF NOT EXISTS isco_commands (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            created_utc TEXT NOT NULL,
            sampler_id TEXT NOT NULL,
            command TEXT NOT NULL,
            reason TEXT,
            executed INTEGER DEFAULT 0,
            response TEXT,
            synced INTEGER DEFAULT 0
        )
    ''')

    cursor.execute('''
        CREATE TABLE IF NOT EXISTS system_alerts (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp_utc TEXT NOT NULL,
            alert_type TEXT NOT NULL,
            sensor_id TEXT NOT NULL,
            value REAL,
            message TEXT NOT NULL,
            synced INTEGER DEFAULT 0
        )
    ''')

    cursor.execute('''
        CREATE TABLE IF NOT EXISTS equipment_events (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp_utc TEXT NOT NULL,
            actuator_name TEXT NOT NULL,
            state TEXT NOT NULL,
            reason TEXT,
            synced INTEGER DEFAULT 0
        )
    ''')

    cursor.execute('''
        CREATE TABLE IF NOT EXISTS sampler_state (
            sampler_id TEXT PRIMARY KEY,
            last_triggered_epoch REAL NOT NULL
        )
    ''')
    conn.commit()

    if using_fallback:
        record_alert(
            cursor,
            alert_type="STORAGE_FALLBACK",
            sensor_id="system_storage",
            value=0.0,
            message=f"SSD not mounted at {MOUNT_POINT}; logging to SD card fallback ({db_path}).",
            cooldown_key="storage_fallback_alert",
        )
        conn.commit()   
    
    return conn


def set_actuator(cursor, relay_device, actuator_name, turn_on, reason=""):
    """Changes relay state and logs ON/OFF transitions to equipment_events."""
    if turn_on and not relay_device.is_active:
        relay_device.on()
        ts = datetime.now(UTC).isoformat()
        cursor.execute(
            'INSERT INTO equipment_events (timestamp_utc, actuator_name, state, reason) VALUES (?, ?, ?, ?)',
            (ts, actuator_name, "ON", reason)
        )
        print(f"[ACTUATOR] {actuator_name} -> ON ({reason})")
    elif not turn_on and relay_device.is_active:
        relay_device.off()
        ts = datetime.now(UTC).isoformat()
        cursor.execute(
            'INSERT INTO equipment_events (timestamp_utc, actuator_name, state, reason) VALUES (?, ?, ?, ?)',
            (ts, actuator_name, "OFF", reason)
        )
        print(f"[ACTUATOR] {actuator_name} -> OFF ({reason})")


def get_last_trigger_time(cursor, key):
    cursor.execute('SELECT last_triggered_epoch FROM sampler_state WHERE sampler_id = ?', (key,))
    row = cursor.fetchone()
    return row[0] if row else 0.0


def set_last_trigger_time(cursor, key, epoch_time):
    cursor.execute('''
        INSERT INTO sampler_state (sampler_id, last_triggered_epoch)
        VALUES (?, ?)
        ON CONFLICT(sampler_id) DO UPDATE SET last_triggered_epoch = excluded.last_triggered_epoch
    ''', (key, epoch_time))


def record_alert(cursor, alert_type, sensor_id, value, message, cooldown_key=None):
    now_epoch = time.time()
    ts = datetime.now(UTC).isoformat()
    if cooldown_key:
        if (now_epoch - get_last_trigger_time(cursor, cooldown_key)) < ALERT_COOLDOWN_SEC:
            return
        set_last_trigger_time(cursor, cooldown_key, now_epoch)

    cursor.execute(
        'INSERT INTO system_alerts (timestamp_utc, alert_type, sensor_id, value, message) VALUES (?, ?, ?, ?, ?)',
        (ts, alert_type, sensor_id, value, message)
    )
    print(f"[ALERT RECORDED] {alert_type}: {message}")
    send_email_notification(f"Station Alert: {alert_type}", f"{message}\nTimestamp: {ts}")


def send_email_notification(subject, body):
    if not SMTP_SERVER:
        return
    try:
        msg = EmailMessage()
        msg['Subject'] = subject
        msg['From'] = SMTP_USER
        msg['To'] = ALERT_RECIPIENT
        msg.set_content(body)
        with smtplib.SMTP(SMTP_SERVER, SMTP_PORT, timeout=10) as server:
            server.starttls()       
            server.login(SMTP_USER, SMTP_PASS)
            server.send_message(msg)
    except (smtplib.SMTPException, OSError) as e:
        print(f"[EMAIL ERROR] {e}")


# ==============================================================================
# SENSOR & SAMPLER POLLING FUNCTIONS
# ==============================================================================


def read_cs451_tank_level():
    try:
        with serial.Serial(SDI12_PORT, baudrate=9600, timeout=2) as ser:
            time.sleep(0.3)
            ser.reset_input_buffer()

            ser.write(f"{CS451_SDI12_ADDR}M!\r\n".encode())
            ser.readline()
            time.sleep(1.5)

            ser.write(f"{CS451_SDI12_ADDR}D0!\r\n".encode())
            raw = ser.readline().decode("utf-8", errors="ignore").strip()

    except serial.SerialException as e:
        print(f"[cs451_tank] Serial port error: {e}")
        return None, None

    # Parse outside the serial block so you don't hold the port open longer than needed
    try:
        values = [float(x) for x in raw[1:].replace("-", "+-").split("+") if x]
    except (ValueError, IndexError) as e:
        print(f"[cs451_tank] Malformed sensor response '{raw}': {e}")
        return None, None

    if len(values) >= 2:
        return values[0], values[1]

    print(f"[cs451_tank] Incomplete data received: {values}")
    return None, None


def poll_sdi12_bus(timestamp):
    rows = []

    # Layer 1: Hardware/port-level failure (stops the whole poll)
    try:
        with serial.Serial(SDI12_PORT, baudrate=9600, timeout=2) as ser:
            time.sleep(0.5)

            for sensor_id, info in SDI12_SENSORS.items():
                addr = info["addr"]
                sensor_type = info["type"]

                # Layer 2: Per-sensor failure (logs and continues to next sensor)
                try:
                    ser.reset_input_buffer()
                    ser.write(f"{addr}M!\r\n".encode())
                    ser.readline()

                    time.sleep(9.0 if sensor_type == "bubbler" else 1.5)

                    ser.write(f"{addr}D0!\r\n".encode())
                    raw = ser.readline().decode("utf-8", errors="ignore").strip()

                    values = [
                        float(x) for x in raw[1:].replace("-", "+-").split("+") if x
                    ]
                except serial.SerialException as e:
                    print(
                        f"[SDI-12] {sensor_id} (addr {addr}) communication error: {e}"
                    )
                    continue
                except ValueError:
                    print(
                        f"[SDI-12] {sensor_id} (addr {addr}) malformed response: '{raw}'"
                    )
                    continue

                if sensor_type == "5tm" and len(values) >= 2:
                    rows.append(
                        (
                            timestamp,
                            sensor_id,
                            values[0], #vwc
                            None,
                            values[1], #temperature_c
                            None,
                            None,
                            None,
                            None,
                            None,
                            None,
                        )
                    )
                elif sensor_type == "bubbler" and len(values) >= 1:
                    rows.append(
                        (
                            timestamp,
                            sensor_id,
                            None,
                            None,
                            None,
                            values[0], #water_level
                            None,
                            None,
                            None,
                            None,
                            None,
                        )
                    )
                else:
                    print(
                        f"[SDI-12] {sensor_id} (addr {addr}) incomplete data: {values}"
                    )

    except serial.SerialException as e:
        print(f"[SDI-12 BUS] Failed to open port {SDI12_PORT}: {e}")

    return rows



def read_modbus_float32(client, register_addr, unit_id):
    try:
        res = client.read_holding_registers(
            address=register_addr,
            count=2,
            slave=unit_id,  # Note: use device_id=unit_id if on pymodbus >= 3.10
        )
    except ModbusException as e:
        raise RuntimeError(
            f"Modbus transport failure at reg {register_addr}, unit {unit_id}: {e}"
        ) from e

    if res is None or res.isError():
        raise RuntimeError(
            f"Modbus protocol error at reg {register_addr}, unit {unit_id}: {res}"
        )

    if len(res.registers) < 2:
        raise ValueError(
            f"Expected 2 registers at {register_addr}, got {len(res.registers)}"
        )

    raw_bytes = struct.pack(">HH", res.registers[0], res.registers[1])
    value = struct.unpack(">f", raw_bytes)[0]

    if not math.isfinite(value):
        raise ValueError(
            f"Invalid float32 ({value}) at reg {register_addr}, unit {unit_id}"
        )

    return round(value, 3)



def poll_sc4500_sensors(timestamp):
    rows = []
    client = ModbusTcpClient(SC4500_IP, port=SC4500_PORT, timeout=2.0)

    try:
        if not client.connect():
            print("[SC4500] Failed to connect via Modbus TCP.")
            return rows

        try:
            turbidity = read_modbus_float32(
                client, SOLITAX_TURBIDITY_REG, SOLITAX_UNIT_ID
            )
            solids = read_modbus_float32(client, SOLITAX_SOLIDS_REG, SOLITAX_UNIT_ID)
            rows.append(
                (
                    timestamp,
                    "hach_solitax",
                    None,
                    None,
                    None,
                    None,
                    turbidity,
                    solids,
                    None,
                    None,
                    None,
                )
            )
        except (RuntimeError, ValueError) as e:
            print(f"[hach_solitax] Read error: {e}")

        try:
            nitrate = read_modbus_float32(
                client, NITRATAX_NITRATE_REG, NITRATAX_UNIT_ID
            )
            rows.append(
                (
                    timestamp,
                    "hach_nitratax",
                    None,
                    None,
                    None,
                    None,
                    None,
                    None,
                    nitrate,
                    None,
                    None,
                )
            )
        except (RuntimeError, ValueError) as e:
            print(f"[hach_nitratax] Read error: {e}")

    finally:
        client.close()

    return rows


# def wake_isco(ser):
#     ser.reset_input_buffer()
#     for _ in range(6):
#         ser.write(b"?")
#         time.sleep(0.25)
#         resp = ser.read_all().decode("utf-8", errors="ignore")
#         if ">" in resp or "6712" in resp:
#             return True
#     return False

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s"
)
logger = logging.getLogger(__name__)

def read_until_prompt(ser: serial.Serial, timeout: float = 2.0) -> str:
    """Reads serial buffer until '>' prompt is seen or timeout expires."""
    deadline = time.monotonic() + timeout
    chunks = []
    while time.monotonic() < deadline:
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting).decode("utf-8", errors="ignore")
            chunks.append(data)
            if ">" in data:
                break
        time.sleep(0.05)
    return "".join(chunks).strip()


def wake_isco(ser: serial.Serial, sampler_id: str) -> bool:
    """Sends '?' up to 6 times to wake the ISCO 6712 command prompt."""
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    for attempt in range(1, 7):
        ser.write(b"?")
        ser.flush()
        resp = read_until_prompt(ser, timeout=0.35)
        if ">" in resp or "6712" in resp:
            logger.debug(f"[{sampler_id}] Woke prompt on attempt {attempt}")
            return True

    return False


def poll_and_control_isco(sampler_id, port_path, timestamp, pending_cmds):
    try:
        with serial.Serial(port_path, 9600, timeout=2) as ser:
            if not wake_isco(ser, sampler_id):
                print(f"[{sampler_id}] Failed to wake ISCO 6712 prompt.")
                return None, [], None

            executed_updates = []
            for cmd_id, cmd_str in pending_cmds:
                ser.write(f"{cmd_str.strip()}\r".encode())
                time.sleep(1.5)
                cmd_resp = ser.read_all().decode("utf-8", errors="ignore").strip()
                executed_updates.append((cmd_resp, cmd_id))
                print(f"[{sampler_id}] Executed '{cmd_str}' -> {cmd_resp}")

            ser.write(b"STS\r")
            time.sleep(1.0)
            sts_raw = ser.read_all().decode("utf-8", errors="ignore").strip()

            bottle_num = None
            for token in sts_raw.replace("\r", ",").replace("\n", ",").split(","):
                token = token.strip()
                if token.startswith("BTL"):
                    digits = "".join(filter(str.isdigit, token))
                    if digits:
                        bottle_num = int(digits)

            row = (
                timestamp,
                sampler_id,
                None,
                None,
                None,
                None,
                None,
                None,
                None,
                bottle_num,
                sts_raw,
            )
            return row, executed_updates, bottle_num

    except (serial.SerialException, OSError, ValueError) as e:
        print(f"[{sampler_id}] ERROR: {e}")
        return None, [], None

def evaluate_trailer_climate(
    cursor, now_epoch: float, trailer_temp_c: float | None
) -> None:
    """Evaluates trailer temperature and safely controls heater/AC relays with hysteresis and compressor protection."""

    # 1. Validate Sensor Input
    if trailer_temp_c is None or isinstance(trailer_temp_c, bool):
        logger.warning("[CLIMATE] Skipping evaluation: trailer_temp_c is None.")
        return

    try:
        temp_c = float(trailer_temp_c)
    except (TypeError, ValueError):
        logger.error(
            f"[CLIMATE] Invalid non-numeric temperature reading: {trailer_temp_c!r}"
        )
        return

    if math.isnan(temp_c) or math.isinf(temp_c):
        logger.warning(f"[CLIMATE] Ignoring NaN/Inf temperature reading: {temp_c}")
        return

    if not (MIN_VALID_TEMP_C <= temp_c <= MAX_VALID_TEMP_C):
        logger.error(
            f"[CLIMATE] Sensor out of plausible range ({temp_c:.2f}C). "
            f"Expected between {MIN_VALID_TEMP_C}C and {MAX_VALID_TEMP_C}C. Skipping control."
        )
        return

    # 2. Sanity-Check Threshold Configuration (Prevent Heater/AC overlap)
    if HEATER_OFF_TEMP_C >= AC_OFF_TEMP_C:
        logger.critical(
            f"[CLIMATE] Configuration error: HEATER_OFF ({HEATER_OFF_TEMP_C}C) >= AC_OFF ({AC_OFF_TEMP_C}C). "
            "Aborting to prevent simultaneous heating/cooling."
        )
        return

    trailer_temp_f = (temp_c * 9.0 / 5.0) + 32.0

    # Safely read relay states (in case GPIO pin factory or hardware throws an error)
    try:
        heater_active = bool(heater_relay.is_active)
        ac_active = bool(ac_relay.is_active)
    except GPIOZeroError:
        logger.exception("[CLIMATE] Failed to read relay GPIO states")
        return

    # Hardware interlock safety check: both relays should never be active at the same time
    if heater_active and ac_active:
        logger.critical(
            "[CLIMATE] SAFETY INTERLOCK: Both Heater and AC relays detected active! Shutting both down."
        )
        _safe_set_actuator(
            cursor, heater_relay, "trailer_heater", False, "Safety interlock shutdown"
        )
        _safe_set_actuator(
            cursor, ac_relay, "trailer_ac", False, "Safety interlock shutdown"
        )
        _safe_set_trigger_time(cursor, "ac_last_off", now_epoch)
        return

    # 3. Heater Control (< 65°F ON, >= 68°F OFF)
    try:
        if temp_c < HEATER_ON_TEMP_C and not heater_active:
            if ac_active:
                _safe_set_actuator(
                    cursor, ac_relay, "trailer_ac", False, "Switching to heating"
                )
                _safe_set_trigger_time(cursor, "ac_last_off", now_epoch)
            _safe_set_actuator(
                cursor,
                heater_relay,
                "trailer_heater",
                True,
                f"Temp {trailer_temp_f:.1f}F < 65F",
            )

        elif temp_c >= HEATER_OFF_TEMP_C and heater_active:
            _safe_set_actuator(
                cursor,
                heater_relay,
                "trailer_heater",
                False,
                f"Temp {trailer_temp_f:.1f}F >= 68F",
            )
    except sqlite3.Error:
        logger.exception("[CLIMATE] Error during heater control evaluation")

    # 4. Air Conditioner Control (> 80°F ON, <= 76°F OFF) with Persistent 5-Min Delay
    try:
        if temp_c > AC_ON_TEMP_C and not ac_active:
            last_ac_off = _safe_get_trigger_time(cursor, "ac_last_off")
            elapsed = now_epoch - last_ac_off

            # Guard against NTP/system clock jumps backwards (elapsed < 0)
            if elapsed < 0:
                logger.warning(
                    f"[CLIMATE] System clock moved backwards (elapsed={elapsed:.1f}s). Resetting ac_last_off."
                )
                _safe_set_trigger_time(cursor, "ac_last_off", now_epoch)
                elapsed = 0

            if elapsed >= AC_MIN_OFF_SECONDS:
                if heater_active:
                    _safe_set_actuator(
                        cursor,
                        heater_relay,
                        "trailer_heater",
                        False,
                        "Switching to cooling",
                    )
                _safe_set_actuator(
                    cursor,
                    ac_relay,
                    "trailer_ac",
                    True,
                    f"Temp {trailer_temp_f:.1f}F > 80F",
                )
            else:
                wait_rem = int(AC_MIN_OFF_SECONDS - elapsed)
                logger.info(
                    f"[CLIMATE] Persistent AC compressor delay active ({wait_rem}s remaining)."
                )

        elif temp_c <= AC_OFF_TEMP_C and ac_active:
            _safe_set_actuator(
                cursor,
                ac_relay,
                "trailer_ac",
                False,
                f"Temp {trailer_temp_f:.1f}F <= 76F",
            )
            _safe_set_trigger_time(cursor, "ac_last_off", now_epoch)
    except sqlite3.Error :
        logger.exception("[CLIMATE] Error during AC control evaluation")

    # 5. Freeze Warning Alert (< 2.0°C)
    if temp_c < FREEZE_ALERT_TEMP_C:
        try:
            record_alert(
                cursor,
                alert_type="FREEZE_WARNING",
                sensor_id=TRAILER_TEMP_SENSOR_ID,
                value=temp_c,
                message=f"Trailer temperature dropped to {temp_c:.2f}C ({trailer_temp_f:.1f}F)",
                cooldown_key="freeze_alert",
            )
        except sqlite3.Error:
            logger.exception("[CLIMATE] Failed to record FREEZE_WARNING alert")


# --- Safe Wrapper Helpers ---


def _safe_set_actuator(cursor, relay, name: str, state: bool, reason: str) -> None:
    try:
        set_actuator(cursor, relay, name, state, reason)
        logger.info(f"[CLIMATE] {name} -> {'ON' if state else 'OFF'} ({reason})")
    except sqlite3.Error as e:
        logger.error(f"[CLIMATE] Failed to set actuator '{name}' to {state}: {e}")
        # Fallback: attempt direct hardware shutoff if turning OFF failed in DB wrapper
        if not state:
            try:
                relay.off()
            except GPIOZeroError as hw_err:
                logger.critical(
                    f"[CLIMATE] CRITICAL: Direct hardware shutoff failed for '{name}': {hw_err}"
                )


def _safe_get_trigger_time(cursor, key: str) -> float:
    try:
        val = get_last_trigger_time(cursor, key)
        return float(val) if val is not None else 0.0
    except sqlite3.Error as e:
        logger.error(
            f"[CLIMATE] DB error reading trigger time '{key}': {e}. Defaulting to 0.0."
        )
        return 0.0


def _safe_set_trigger_time(cursor, key: str, epoch: float) -> None:
    try:
        set_last_trigger_time(cursor, key, epoch)
    except sqlite3.Error as e:
        logger.error(f"[CLIMATE] DB error writing trigger time '{key}'={epoch}: {e}")
        
        
        
        
def main():
    conn = init_db()
    cursor = conn.cursor()

    print(
        "Starting Station Controller "
        "(Tank Fill, Rinse/Backflush, Dry-Run Guard, Climate & ISCO)..."
    )

    while True:
        cycle_start = time.time()

        try:
            # 1. Ensure rinse valve and 1" drain valve are closed before filling
            set_actuator(
                cursor,
                rinse_valve,
                "tank_rinse_valve",
                False,
                "Closing before tank fill",
            )
            set_actuator(
                cursor,
                drain_valve,
                "tank_drain_valve",
                False,
                "Closing before tank fill",
            )
            conn.commit()

            time.sleep(5.0)

            # 2. Check high-level float switch before starting pump
            if not high_float_switch.is_pressed:
                record_alert(
                    cursor,
                    "TANK_HIGH_FLOAT",
                    "high_float_switch",
                    1.0,
                    "Tank high-level float switch tripped before fill! "
                    "Skipping pump cycle.",
                    cooldown_key="high_float_alert",
                )
            else:
                # 3. Fill tank with dry-run/stall detection and high-float cutoff
                set_actuator(
                    cursor,
                    pump_relay,
                    "stream_pump",
                    True,
                    "Filling sampling tank",
                )
                conn.commit()

                fill_start = time.time()
                last_check_time = fill_start
                baseline_level, _ = read_cs451_tank_level()

                while (time.time() - fill_start) < MAX_FILL_SECONDS:
                    if not high_float_switch.is_pressed:
                        set_actuator(
                            cursor,
                            pump_relay,
                            "stream_pump",
                            False,
                            "EMERGENCY: High-level float tripped",
                        )
                        record_alert(
                            cursor,
                            "TANK_HIGH_FLOAT",
                            "high_float_switch",
                            1.0,
                            "Tank high-level float switch tripped during fill! "
                            "Pump stopped immediately.",
                        )
                        break

                    tank_level, _ = read_cs451_tank_level()
                    now = time.time()

                    if tank_level is not None:
                        print(f"[TANK] CS451 Level: {tank_level:.3f} ft")

                        if tank_level >= TARGET_FILL_LEVEL:
                            set_actuator(
                                cursor,
                                pump_relay,
                                "stream_pump",
                                False,
                                f"Target level {tank_level:.2f}ft reached",
                            )
                            break

                    # Dry-run/clogged-intake check every 15 seconds
                    if (now - last_check_time) >= DRY_RUN_CHECK_INTERVAL:
                        if baseline_level is not None and tank_level is not None:
                            gain = tank_level - baseline_level

                            if gain < MIN_LEVEL_GAIN_PER_CHECK:
                                set_actuator(
                                    cursor,
                                    pump_relay,
                                    "stream_pump",
                                    False,
                                    "ABORT: Pump dry-run / stall detected",
                                )
                                record_alert(
                                    cursor,
                                    "PUMP_DRY_RUN",
                                    "cs451_tank",
                                    gain,
                                    f"Pump stalled/dry-run: tank rose only "
                                    f"{gain:.3f}ft in "
                                    f"{DRY_RUN_CHECK_INTERVAL}s",
                                    cooldown_key="pump_dry_run_alert",
                                )
                                break

                        baseline_level = tank_level
                        last_check_time = now

                    time.sleep(3.0)
                else:
                    set_actuator(
                        cursor,
                        pump_relay,
                        "stream_pump",
                        False,
                        "Max fill timeout reached",
                    )

                conn.commit()
                time.sleep(2.0)

            # 4. Poll all sensors concurrently
            now_epoch = time.time()
            timestamp = datetime.now(UTC).isoformat()
            rows_to_insert = []

            tank_level, tank_temp = read_cs451_tank_level()
            if tank_level is not None:
                rows_to_insert.append(
                    (
                        timestamp,
                        "cs451_tank",
                        None,
                        None,
                        tank_temp,
                        tank_level,
                        None,
                        None,
                        None,
                        None,
                        None,
                    )
                )

            with ThreadPoolExecutor(max_workers=2) as executor:
                futures = [executor.submit(poll_sdi12_bus, timestamp),
                           executor.submit(poll_sc4500_sensors, timestamp)]
            
                for future in as_completed(futures):
                    result = future.result()

                    if isinstance(result, list):
                        rows_to_insert.extend(result)
                    elif result is not None:
                        rows_to_insert.append(result)

            stream_water_level = None
            current_turbidity = None
            trailer_temp_c = None

            for row in rows_to_insert:
                if row[1] == "ysi_bubbler":
                    stream_water_level = row[5]
                elif row == "hach_solitax":
                    current_turbidity = row[6]
                elif row[1] == TRAILER_TEMP_SENSOR_ID:
                    trailer_temp_c = row[4]

            # 5. Evaluate trailer heater, AC, and freeze alert
            evaluate_trailer_climate(cursor, now_epoch, trailer_temp_c)

            # 6. Evaluate automated ISCO 6712 trigger rules
            if (
                stream_water_level is not None
                and stream_water_level > ISCO_01_LEVEL_THRESHOLD
                and now_epoch - get_last_trigger_time(cursor, "isco_01")
                >= ISCO_01_COOLDOWN_SEC
            ):
                reason = (
                    f"Stream flow active: {stream_water_level} ft > "
                    f"{ISCO_01_LEVEL_THRESHOLD} ft (90m interval)"
                )
                cursor.execute(
                    """
                    INSERT INTO isco_commands (
                        created_utc, sampler_id, command, reason
                    ) VALUES (?, ?, ?, ?)
                    """,
                    (timestamp, "isco_01", "TAKE", reason),
                )
                set_last_trigger_time(cursor, "isco_01", now_epoch)

            if (
                stream_water_level is not None
                and current_turbidity is not None
                and stream_water_level > ISCO_02_LEVEL_THRESHOLD
                and current_turbidity > ISCO_02_TURBIDITY_THRESHOLD
                and (
                    now_epoch - get_last_trigger_time(cursor, "isco_02")
                ) >= ISCO_02_COOLDOWN_SEC
            ):
                reason = (
                    f"Storm event: stream level {stream_water_level} ft "
                    f"> {ISCO_02_LEVEL_THRESHOLD} ft AND turbidity "
                    f"{current_turbidity} > "
                    f"{ISCO_02_TURBIDITY_THRESHOLD}"
                )
                cursor.execute(
                    """
                    INSERT INTO isco_commands (
                        created_utc, sampler_id, command, reason
                    ) VALUES (?, ?, ?, ?)
                    """,
                    (timestamp, "isco_02", "TAKE", reason),
                )
                set_last_trigger_time(cursor, "isco_02", now_epoch)

            conn.commit()

            # 7. Poll ISCO 6712 autosamplers and track bottle count
            with ThreadPoolExecutor(max_workers=len(ISCO_SAMPLERS)) as executor:
                isco_futures = {}

                for sampler_id, port in ISCO_SAMPLERS.items():
                    cursor.execute(
                        """
                        SELECT id, command
                        FROM isco_commands
                        WHERE sampler_id = ? AND executed = 0
                        ORDER BY id ASC
                        """,
                        (sampler_id,),
                    )
                    pending = cursor.fetchall()

                    future = executor.submit(
                        poll_and_control_isco,
                        sampler_id,
                        port,
                        timestamp,
                        pending,
                    )
                    isco_futures[future] = sampler_id

                for future in as_completed(isco_futures):
                    sampler_id = isco_futures[future]
                    isco_row, executed_updates, bottle_num = future.result()

                    if isco_row is not None:
                        rows_to_insert.append(isco_row)

                    for cmd_resp, cmd_id in executed_updates:
                        cursor.execute(
                            """
                            UPDATE isco_commands
                            SET executed = 1, response = ?
                            WHERE id = ?
                            """,
                            (cmd_resp, cmd_id),
                        )

                    if (
                        bottle_num is not None
                        and bottle_num >= ISCO_BOTTLE_WARN_THRESHOLD
                    ):
                        record_alert(
                            cursor,
                            "ISCO_BOTTLE_FULL_SOON",
                            sampler_id,
                            float(bottle_num),
                            f"{sampler_id} is on bottle {bottle_num} of 24! "
                            "Schedule bottle carousel swap.",
                            cooldown_key=f"{sampler_id}_bottle_warn",
                        )

            # 8. Batch commit sensor readings
            if rows_to_insert:
                cursor.executemany(
                    """
                    INSERT INTO soil_readings (
                        timestamp_utc,
                        sensor_id,
                        vwc,
                        ec,
                        temperature_c,
                        water_level,
                        turbidity,
                        suspended_solids,
                        nitrate_mg_l,
                        isco_bottle,
                        isco_status
                    ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                    """,
                    rows_to_insert,
                )
                conn.commit()

                print(f"[{timestamp}] Saved {len(rows_to_insert)} records to SSD.")

        finally:
            # 9. End-of-cycle drain and auto-rinse/backflush routine
            set_actuator(
                cursor,
                pump_relay,
                "stream_pump",
                False,
                "End of measurement cycle",
            )
            set_actuator(
                cursor,
                drain_valve,
                "tank_drain_valve",
                True,
                'Opening 1" drain valve',
            )
            set_actuator(
                cursor,
                rinse_valve,
                "tank_rinse_valve",
                True,
                f"Auto-rinse/backflush for {RINSE_DURATION_SECONDS}s",
            )
            conn.commit()

            time.sleep(RINSE_DURATION_SECONDS)

            set_actuator(
                cursor,
                rinse_valve,
                "tank_rinse_valve",
                False,
                "Auto-rinse/backflush complete",
            )
            conn.commit()

            remaining_drain = max(
                0,
                DRAIN_DURATION_SECONDS - RINSE_DURATION_SECONDS,
            )
            time.sleep(remaining_drain)

            set_actuator(
                cursor,
                drain_valve,
                "tank_drain_valve",
                False,
                "Drain complete",
            )
            conn.commit()

            elapsed = time.time() - cycle_start
            time.sleep(max(0, LOG_INTERVAL_SECONDS - elapsed))


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pump_relay.off()
        drain_valve.off()
        rinse_valve.off()
        heater_relay.off()
        ac_relay.off()

        print("\nSystem stopped safely (all relays OFF).")