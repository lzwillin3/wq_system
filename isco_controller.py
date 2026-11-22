import argparse
import csv
import json
import logging
import os
import sys
import threading
import time
from datetime import datetime

import serial
from gpiozero import LED, Button

# --- Hardware & Serial Configuration ---
SERIAL_PORT = "/dev/serial0"
BAUD_RATE = 9600

RESET_BUTTON_GPIO = 17    # Physical Pin 11 -> GND (Resets bottle counter to #1)
TRIGGER_BUTTON_GPIO = 27  # Physical Pin 13 -> GND (Triggers next sample)
GREEN_LED_GPIO = 22       # Physical Pin 15 -> 330Ω -> LED -> GND (Ready / Blinking = Pumping)
RED_LED_GPIO = 23         # Physical Pin 16 -> 330Ω -> LED -> GND (Rack Full / Error)

# --- Sampler Rack Configuration ---
TOTAL_BOTTLES = 24          # 1, 2, 4, 8, 12, or 24 depending on your 6712 distributor rack
BOTTLE_CAPACITY_ML = 1000   # Max mL per bottle before advancing to the next bottle

STATE_FILE = "bottle_state.json"
LOG_FILE = "isco_sampler.log"
CSV_FILE = "isco_samples.csv"

CSV_HEADERS = [
    "timestamp",
    "trigger_source",
    "bottle_num",
    "bottle_volume_after_ml",
    "requested_volume_ml",
    "actual_volume_ml",
    "attempt",
    "success",
    "sor_code",
    "sor_description",
    "isco_id",
    "isco_time",
    "raw_response",
]

# Create a logger specific to this module
logger = logging.getLogger(__name__)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[
        logging.FileHandler(LOG_FILE),
        logging.StreamHandler(),
    ],
)

SOR_CODES = {
    0: ("Sample OK", False),
    1: ("No liquid detected", True),
    2: ("Liquid lost during sample", True),
    3: ("Pump timeout", True),
    4: ("Power failed during sample", True),
    5: ("Pump jammed (Hardware fault - check pump)", False),
    6: ("Distributor arm jammed (Hardware fault - check arm)", False),
    7: ("No distributor detected", False),
    8: ("Pump latch open (Hardware fault - close latch)", False),
    9: ("Sample disabled / Program stopped", False),
    10: ("Bottle full", False),
    11: ("User stopped sample", False),
    12: ("Invalid bottle or volume parameter", False),
}


class StatusIndicator:
    """Manages Green (Ready/Active) and Red (Rack Full/Error) status LEDs."""

    def __init__(self, green_pin: int, red_pin: int):
        self.green = LED(green_pin)
        self.red = LED(red_pin)
        self.has_error = False

    def set_ready(self, rack_full: bool = False):
        """Set steady LED state based on whether the rack has available bottles."""
        self.green.off()
        self.red.off()
        if rack_full or self.has_error:
            self.red.on()
        else:
            self.green.on()

    def set_pumping(self):
        """Blink green LED while a sample cycle is actively running."""
        self.red.off()
        self.green.blink(on_time=1, off_time=1, background=True)

   
    def set_error(self):
        """Turn on solid red LED for hardware/serial faults or full rack."""
        self.has_error = True
        self.green.off()
        self.red.on()

    def clear_error(self, rack_full: bool = False):
        """Clear error state (e.g., when reset switch is pressed)."""
        self.has_error = False
        self.set_ready(rack_full=rack_full)

    def cleanup(self):
        self.green.off()
        self.red.off()


class BottleTracker:
    """Thread-safe persistent bottle counter with GPIO switch reset support."""

    def __init__(
        self,
        state_file: str,
        total_bottles: int,
        bottle_capacity_ml: int,
        indicator: StatusIndicator | None = None,
    ):
        self.state_file = state_file
        self.total_bottles = total_bottles
        self.bottle_capacity_ml = bottle_capacity_ml
        self.indicator = indicator
        self.lock = threading.Lock()
        self.state = self._load_state()
        if self.indicator:
            self.indicator.set_ready(rack_full=self.state.get("rack_full", False))

    def _load_state(self) -> dict:
        if os.path.isfile(self.state_file):
            try:
                with open(self.state_file, "r", encoding="utf-8") as f:
                    data = json.load(f)
                    if "current_bottle" in data and "current_volume_ml" in data:
                        return data
            except (json.JSONDecodeError, OSError) as e:
                logger.warning(f"Corrupt state file, reinitializing: {e}")

        default_state = {
            "current_bottle": 1,
            "current_volume_ml": 0,
            "rack_full": False,
            "last_reset": datetime.now().isoformat(timespec="seconds"),
        }
        self._save_state(default_state)
        return default_state

    def _save_state(self, state_data: dict):
        with open(self.state_file, "w", encoding="utf-8") as f:
            json.dump(state_data, f, indent=2)

    @property
    def is_rack_full(self) -> bool:
        with self.lock:
            return bool(self.state.get("rack_full", False))

    def reset(self):
        """Reset tracking to Bottle #1 and clear any error/full LED states."""
        with self.lock:
            self.state = {
                "current_bottle": 1,
                "current_volume_ml": 0,
                "rack_full": False,
                "last_reset": datetime.now().isoformat(timespec="seconds"),
            }
            self._save_state(self.state)
            logger.info("BOTTLE TRACKER RESET: Ready at Bottle #1 (0 mL).")
        if self.indicator:
            self.indicator.clear_error(rack_full=False)

    def get_target_bottle(self, requested_ml: int) -> tuple[int | None, int]:
        with self.lock:
            if self.state.get("rack_full"):
                return None, 0

            bottle = self.state["current_bottle"]
            vol = self.state["current_volume_ml"]

            if vol > 0 and (vol + requested_ml > self.bottle_capacity_ml):
                if bottle >= self.total_bottles:
                    self.state["rack_full"] = True
                    self._save_state(self.state)
                    return None, 0
                bottle += 1
                vol = 0
                self.state["current_bottle"] = bottle
                self.state["current_volume_ml"] = 0
                self._save_state(self.state)
                logger.info(f"Auto-advanced to Bottle #{bottle} to prevent overflow.")

            return bottle, vol
    def record_sample(self, deposited_ml: int, force_advance: bool = False) -> int:
        with self.lock:
            self.state["current_volume_ml"] += deposited_ml
            new_vol = self.state["current_volume_ml"]

            if force_advance or new_vol >= self.bottle_capacity_ml:
                if self.state["current_bottle"] >= self.total_bottles:
                    self.state["rack_full"] = True
                    logger.warning(
                        f"RACK FULL: All {self.total_bottles} bottles filled! Replace bottles and press reset switch."
                    )
                else:
                    self.state["current_bottle"] += 1
                    self.state["current_volume_ml"] = 0
                    logger.info(
                        f"Bottle complete. Next sample will go into Bottle #{self.state['current_bottle']}."
                    )

            self._save_state(self.state)
            return new_vol



def log_to_csv(row: dict):
    file_exists = os.path.isfile(CSV_FILE) and os.path.getsize(CSV_FILE) > 0
    with open(CSV_FILE, mode="a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_HEADERS)
        if not file_exists:
            writer.writeheader()
        writer.writerow(row)


def calc_checksum(payload: str) -> int:
    return sum(payload.encode("ascii"))


def parse_isco_response(raw: str) -> dict:
    parts = [p.strip() for p in raw.split(",") if p.strip()]
    parsed = {}
    for i in range(0, len(parts) - 1, 2):
        key = parts[i]
        val = parts[i + 1]
        parsed[key] = int(val) if val.isdigit() else val
    return parsed


def send_isco_command(
    ser: serial.Serial, cmd_body: str, use_checksum: bool = True
) -> tuple[dict, str]:
    if use_checksum:
        prefix = f"{cmd_body},CS,"
        cs = calc_checksum(prefix)
        full_cmd = f"{prefix}{cs}\r"
    else:
        full_cmd = f"{cmd_body}\r"

    ser.reset_input_buffer()
    ser.write(full_cmd.encode("ascii"))
    ser.flush()

    raw_response = ser.read_until(b"\r").decode("ascii", errors="replace").strip()
    if not raw_response:
        raise TimeoutError(f"Timed out waiting for response to command: {cmd_body}")

    return parse_isco_response(raw_response), raw_response


def wake_sampler(ser: serial.Serial, max_attempts: int = 10) -> bool:
    for attempt in range(1, max_attempts + 1):
        ser.write(b"?")
        ser.flush()
        time.sleep(0.5)
        if ser.in_waiting > 0:
            banner = ser.read(ser.in_waiting).decode("ascii", errors="replace").strip()
            logger.info(f"Sampler awake on attempt {attempt}: {banner}")
            return True
    return False


sampler_busy_lock = threading.Lock()

def take_tracked_sample(
    tracker: BottleTracker,
    indicator: StatusIndicator,
    volume_ml: int,
    trigger_source: str = "CLI",
    one_sample_per_bottle: bool = False,
    max_retries: int = 3,
    retry_delay_sec: int = 10,
    cycle_timeout_sec: float = 90.0,
) -> bool:
    """Trigger a sample into the next tracked bottle and update LEDs, state, and CSV."""
    if not sampler_busy_lock.acquire(blocking=False):
        logger.warning(
            f"Ignored {trigger_source} trigger: Sampler is currently busy running a sample cycle."
        )
        return False

    try:
        bottle_num, current_vol = tracker.get_target_bottle(volume_ml)
        if bottle_num is None:
            logger.error(
                "Sample skipped: Rack is marked FULL. Install empty bottles and press the GPIO reset switch."
            )
            indicator.set_ready(rack_full=True)
            return False

        indicator.set_pumping()
        logger.info(
            f"[{trigger_source}] Targeting Bottle #{bottle_num} "
            f"(Current volume: {current_vol}/{tracker.bottle_capacity_ml} mL)"
        )

        with serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUD_RATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=cycle_timeout_sec,
        ) as ser:
            if not wake_sampler(ser):
                msg = "Failed to wake ISCO 6712"
                logger.error(msg)
                log_to_csv(
                    {
                        "timestamp": datetime.now().isoformat(timespec="seconds"),
                        "trigger_source": trigger_source,
                        "bottle_num": bottle_num,
                        "bottle_volume_after_ml": current_vol,
                        "requested_volume_ml": volume_ml,
                        "actual_volume_ml": 0,
                        "attempt": 0,
                        "success": False,
                        "sor_code": "NO_WAKE",
                        "sor_description": msg,
                        "isco_id": "",
                        "isco_time": "",
                        "raw_response": "",
                    }
                )
                indicator.set_error()
                return False

            status_resp, _ = send_isco_command(ser, "STS,2")
            logger.info(f"Sampler status: STS={status_resp.get('STS', 'UNKNOWN')}")

            for attempt in range(1, max_retries + 1):
                cmd = f"BTL,{bottle_num},SVO,{volume_ml}"
                logger.info(f"Attempt {attempt}/{max_retries}: Sending {cmd}")
                attempt_ts = datetime.now().isoformat(timespec="seconds")

                try:
                    resp, raw_resp = send_isco_command(ser, cmd)
                except TimeoutError as e:
                    logger.warning(f"Serial timeout on attempt {attempt}: {e}")
                    log_to_csv(
                        {
                            "timestamp": attempt_ts,
                            "trigger_source": trigger_source,
                            "bottle_num": bottle_num,
                            "bottle_volume_after_ml": current_vol,
                            "requested_volume_ml": volume_ml,
                            "actual_volume_ml": 0,
                            "attempt": attempt,
                            "success": False,
                            "sor_code": "TIMEOUT",
                            "sor_description": str(e),
                            "isco_id": status_resp.get("ID", ""),
                            "isco_time": "",
                            "raw_response": "",
                        }
                    )
                    if attempt < max_retries:
                        time.sleep(retry_delay_sec)
                    continue

                sor_code_raw = resp.get("SOR")
                sor_code = sor_code_raw if isinstance(sor_code_raw, int) else -1
                description, is_recoverable = SOR_CODES.get(
                    sor_code, (f"Unknown SOR code ({sor_code})", False)
                )
                is_success = sor_code == 0
                actual_vol = resp.get("SVO", volume_ml if is_success else 0)

                if is_success:
                    updated_vol = tracker.record_sample(
                        deposited_ml=actual_vol,
                        force_advance=one_sample_per_bottle,
                    )
                elif sor_code == 10:  # Bottle full / overflow error code on ISCO
                    updated_vol = tracker.record_sample(
                        deposited_ml=0, force_advance=True
                    )
                else:
                    updated_vol = current_vol

                log_to_csv(
                    {
                        "timestamp": attempt_ts,
                        "trigger_source": trigger_source,
                        "bottle_num": resp.get("BTL", bottle_num),
                        "bottle_volume_after_ml": updated_vol,
                        "requested_volume_ml": volume_ml,
                        "actual_volume_ml": actual_vol,
                        "attempt": attempt,
                        "success": is_success,
                        "sor_code": sor_code,
                        "sor_description": description,
                        "isco_id": resp.get("ID", ""),
                        "isco_time": resp.get("TI", ""),
                        "raw_response": raw_resp,
                    }
                )

                if is_success:
                    logger.info(
                        f"SUCCESS: {actual_vol} mL placed in Bottle #{bottle_num} "
                        f"(Total in bottle: {updated_vol} mL)."
                    )
                    indicator.clear_error(rack_full=tracker.is_rack_full)
                    return True

                logger.warning(f"Sample failed (SOR={sor_code}: {description})")
                if not is_recoverable:
                    logger.error("Non-recoverable fault. Aborting retries.")
                    indicator.set_error()
                    return False

                if attempt < max_retries:
                    logger.info(f"Retrying in {retry_delay_sec} seconds...")
                    time.sleep(retry_delay_sec)

            logger.error(f"All {max_retries} attempts failed for Bottle #{bottle_num}.")
            indicator.set_error()
            return False

    except Exception:
        logger.exception("Unexpected error during sample cycle")
        indicator.set_error()
        return False
    finally:
        sampler_busy_lock.release()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ISCO 6712 Dual-Button & LED Controller")
    parser.add_argument(
        "--volume", type=int, default=150, help="Sample volume in mL (default: 150)"
    )
    parser.add_argument(
        "--single-per-bottle",
        action="store_true",
        help="Advance to the next bottle after every successful sample (discrete mode)",
    )
    parser.add_argument(
        "--reset",
        action="store_true",
        help="Manually reset bottle counter to Bottle #1 before running",
    )
    parser.add_argument(
        "--daemon",
        action="store_true",
        help="Run continuously, listening to both GPIO trigger and reset switches",
    )
    args = parser.parse_args()

    indicator = StatusIndicator(green_pin=GREEN_LED_GPIO, red_pin=RED_LED_GPIO)
    tracker = BottleTracker(
        state_file=STATE_FILE,
        total_bottles=TOTAL_BOTTLES,
        bottle_capacity_ml=BOTTLE_CAPACITY_ML,
        indicator=indicator,
    )

    reset_button = Button(RESET_BUTTON_GPIO, pull_up=True, bounce_time=0.2)
    trigger_button = Button(TRIGGER_BUTTON_GPIO, pull_up=True, bounce_time=0.2)

    reset_button.when_pressed = tracker.reset
    if args.reset or bool(getattr(reset_button, "is_pressed", False)):
        tracker.reset()

    def on_trigger_pressed():
        threading.Thread(
            target=take_tracked_sample,
            kwargs={
                "tracker": tracker,
                "indicator": indicator,
                "volume_ml": args.volume,
                "trigger_source": "GPIO_TRIGGER",
                "one_sample_per_bottle": args.single_per_bottle,
            },
            daemon=True,
        ).start()

    trigger_button.when_pressed = on_trigger_pressed

    if args.daemon:
        logger.info(
            f"Daemon active: Green LED=GPIO {GREEN_LED_GPIO}, Red LED=GPIO {RED_LED_GPIO}, "
            f"Trigger=GPIO {TRIGGER_BUTTON_GPIO}, Reset=GPIO {RESET_BUTTON_GPIO}."
        )
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            logger.info("Shutting down daemon.")
        finally:
            indicator.cleanup()
    else:
        success = take_tracked_sample(
            tracker=tracker,
            indicator=indicator,
            volume_ml=args.volume,
            trigger_source="CLI",
            one_sample_per_bottle=args.single_per_bottle,
        )
        if not success:
            sys.exit()
