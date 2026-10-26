"""
LiteWing ESC motor bench test
=============================

Tests each ESC/brushless motor one at a time after flashing the
ESC version of motors.c / motors.h.

How it works:
  The firmware has built-in parameters "motorPowerSet.m1..m4" and
  "motorPowerSet.enable". When enable = 1, the flight controller (PID)
  and mixer are bypassed and each motor gets exactly the thrust we set.
  The new firmware turns that thrust into a 1000-2000 us ESC pulse.

SAFETY - READ BEFORE RUNNING:
  1. REMOVE ALL PROPELLERS.
  2. Brushed motors must be unplugged from the LiteWing board.
  3. Keep the drone upright (upside down triggers tumble protection).
  4. In motorPowerSet mode there is NO lost-connection failsafe:
     if this script crashes or Wi-Fi drops while a motor is spinning,
     the motor KEEPS spinning. Keep a hand near the ESC battery plug.

Usage:
  python esc_motor_test.py        -> interactive menu (needs a real terminal)
  python esc_motor_test.py 4      -> run test 4 directly, no typing needed
                                     (1-4 single motor, 5 sequence, 6 all, 7 ramp)

Requirements:
  pip install cflib
  Connect your PC to the drone's Wi-Fi (LiteWing_xxxx, password 12345678)
"""

import sys
import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig

# ------------------------------------------------------------------
# Settings
# ------------------------------------------------------------------
DRONE_URI = "udp://192.168.0.07:2390" #Caustom Drone(made by us)

TEST_PULSE_US = 1100     # pulse used for single-motor tests (1100 = ~10% throttle)
MAX_PULSE_US = 1300      # safety cap: this script never sends more than this
SPIN_TIME_S = 3.0        # how long each motor spins in a test
RAMP_STEP_US = 25        # ramp test step size
RAMP_STEP_TIME_S = 0.5   # time spent on each ramp step

# Test chosen on the command line (e.g. "python esc_motor_test.py 4"), or None for menu
AUTO_CHOICE = sys.argv[1] if len(sys.argv) > 1 else None
AUTO_COUNTDOWN_S = 5     # seconds of warning before an automatic test starts

MOTOR_NAMES = {
    1: "M1 (front-right, IO5)",
    2: "M2 (back-right,  IO6)",
    3: "M3 (back-left,   IO3)",
    4: "M4 (front-left,  IO4)",
}


# ------------------------------------------------------------------
# Conversion: pulse width (us) <-> firmware thrust (0..65535)
# Same formula as thrustToPulseUs() in the new motors.c, reversed.
# ------------------------------------------------------------------
def us_to_thrust(pulse_us):
    pulse_us = max(1000, min(pulse_us, MAX_PULSE_US))   # enforce safety cap
    return int(round((pulse_us - 1000) * 65535 / 1000))


# ------------------------------------------------------------------
# Motor control helpers
# ------------------------------------------------------------------
def set_motor(cf, motor_number, pulse_us):
    """Set one motor (1-4) to a pulse width in microseconds."""
    thrust = us_to_thrust(pulse_us)
    cf.param.set_value(f"motorPowerSet.m{motor_number}", str(thrust))


def set_all_motors(cf, pulse_us):
    for m in range(1, 5):
        set_motor(cf, m, pulse_us)


def enable_direct_mode(cf):
    """All motors to 1000 us first, THEN bypass the flight controller."""
    set_all_motors(cf, 1000)
    time.sleep(0.2)
    cf.param.set_value("motorPowerSet.enable", "1")
    time.sleep(0.2)


def all_off(cf):
    """Stop everything and give control back to the flight controller."""
    set_all_motors(cf, 1000)
    time.sleep(0.2)
    cf.param.set_value("motorPowerSet.enable", "0")
    time.sleep(0.2)


# ------------------------------------------------------------------
# Live readout of the pulse the firmware is actually sending
# ------------------------------------------------------------------
def start_pulse_log(cf):
    log_conf = LogConfig(name="ESC pulses", period_in_ms=200)
    for m in range(1, 5):
        log_conf.add_variable(f"pwm.m{m}_us", "uint32_t")

    last_values = {}

    def on_data(timestamp, data, logconf):
        values = tuple(data[f"pwm.m{m}_us"] for m in range(1, 5))
        if values != last_values.get("v"):          # print only when something changes
            last_values["v"] = values
            text = "  ".join(f"M{m}={v}us" for m, v in zip(range(1, 5), values))
            print(f"\n  [firmware output] {text}", flush=True)

    log_conf.data_received_cb.add_callback(on_data)
    cf.log.add_config(log_conf)
    log_conf.start()
    return log_conf


def check_new_firmware(cf):
    """The ESC firmware adds 'pwm.m1_us'. If it's missing, the old firmware is running."""
    element = cf.log.toc.get_element_by_complete_name("pwm.m1_us")
    return element is not None


# ------------------------------------------------------------------
# Tests
# ------------------------------------------------------------------
def confirm(message):
    if AUTO_CHOICE is not None:
        # No keyboard available: give a countdown instead of asking
        print(f"\n{message}")
        print("PROPS MUST BE REMOVED. Press Ctrl+C / Stop to cancel.")
        for i in range(AUTO_COUNTDOWN_S, 0, -1):
            print(f"  starting in {i}...", flush=True)
            time.sleep(1)
        return True
    answer = input(f"\n{message}\nProps removed? Press Enter to start (or type n to cancel): ")
    return answer.strip().lower() != "n"


def test_single_motor(cf, motor_number):
    if not confirm(f"Spin {MOTOR_NAMES[motor_number]} at {TEST_PULSE_US} us for {SPIN_TIME_S} s."):
        return
    set_motor(cf, motor_number, TEST_PULSE_US)
    time.sleep(SPIN_TIME_S)
    set_motor(cf, motor_number, 1000)
    print(f"\n  {MOTOR_NAMES[motor_number]} stopped.")
    print("  Check: did the RIGHT motor spin? Which direction (CW / CCW)?")


def test_sequence(cf):
    if not confirm("Spin M1 -> M2 -> M3 -> M4, one after another."):
        return
    for m in range(1, 5):
        print(f"\n  >>> {MOTOR_NAMES[m]}")
        set_motor(cf, m, TEST_PULSE_US)
        time.sleep(SPIN_TIME_S)
        set_motor(cf, m, 1000)
        time.sleep(1.0)
    print("\n  Sequence done.")


def test_all_together(cf):
    if not confirm(f"Spin ALL motors together at {TEST_PULSE_US} us for {SPIN_TIME_S} s."):
        return
    set_all_motors(cf, TEST_PULSE_US)
    time.sleep(SPIN_TIME_S)
    set_all_motors(cf, 1000)
    print("\n  All stopped.")


def test_ramp(cf):
    if not confirm(f"Ramp ALL motors 1000 -> {MAX_PULSE_US} us and back down."):
        return
    up = list(range(1000, MAX_PULSE_US + 1, RAMP_STEP_US))
    for pulse in up + up[::-1]:
        set_all_motors(cf, pulse)
        time.sleep(RAMP_STEP_TIME_S)
    set_all_motors(cf, 1000)
    print("\n  Ramp done. Motors should have sped up and slowed down smoothly.")


def run_choice(cf, choice):
    """Run one test. Returns False if the choice was 'quit'."""
    if choice in ("1", "2", "3", "4"):
        test_single_motor(cf, int(choice))
    elif choice == "5":
        test_sequence(cf)
    elif choice == "6":
        test_all_together(cf)
    elif choice == "7":
        test_ramp(cf)
    elif choice == "0":
        return False
    else:
        print(f"Unknown choice: {choice}")
    return True


# ------------------------------------------------------------------
# Main
# ------------------------------------------------------------------
def main():
    cflib.crtp.init_drivers()
    print(f"Connecting to {DRONE_URI} ...")

    with SyncCrazyflie(DRONE_URI, cf=Crazyflie(rw_cache="./cache")) as scf:
        cf = scf.cf
        print("Connected.")

        if not check_new_firmware(cf):
            print("\n'pwm.m1_us' not found: the drone is still running the OLD firmware.")
            print("Flash the ESC version of motors.c / motors.h first. Exiting.")
            return

        log_conf = start_pulse_log(cf)

        try:
            enable_direct_mode(cf)
            print("\nDirect motor mode ON (flight controller bypassed). All ESCs at 1000 us.")

            if AUTO_CHOICE is not None:
                # Command-line mode: run the one test, then stop
                run_choice(cf, AUTO_CHOICE)
                time.sleep(1.0)
            else:
                while True:
                    print("\n\n========== ESC MOTOR TEST ==========")
                    for m in range(1, 5):
                        print(f"  {m}  Test {MOTOR_NAMES[m]}")
                    print("  5  Sequence M1 -> M4")
                    print("  6  All motors together")
                    print(f"  7  Ramp all motors 1000 -> {MAX_PULSE_US} us")
                    print("  0  Quit")
                    choice = input("Choose: ").strip()
                    if not run_choice(cf, choice):
                        break

        except KeyboardInterrupt:
            print("\nCtrl+C pressed.")

        finally:
            # Always runs, even after an error or Ctrl+C
            print("\nStopping all motors and restoring normal control...")
            all_off(cf)
            log_conf.stop()
            print("Done. Motors stopped, motorPowerSet.enable = 0.")


if __name__ == "__main__":
    main()