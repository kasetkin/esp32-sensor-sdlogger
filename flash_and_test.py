#!/usr/bin/env python3
"""Flash test firmware to ESP32-C6 and capture all serial output.

Device output goes to stdout; all script diagnostics go to stderr.
This means output capture and result analysis can be done by the caller:

    python run_tests.py > output.txt 2>flash.log
    python run_tests.py | grep -c PASS

Exit codes:
    0  — flash succeeded (or skipped) and serial read completed
    1  — flash failed or serial port error
"""

import argparse
import os
import subprocess
import sys
import time

try:
    import esptool  # noqa: F401
except ImportError:
    _idf_python = os.path.join(
        os.environ.get("IDF_PYTHON_ENV_PATH", ""), "bin", "python"
    )
    if os.path.exists(_idf_python) and sys.executable != _idf_python:
        os.execv(_idf_python, [_idf_python] + sys.argv)
    print("[run_tests] esptool not found and ESP-IDF Python not available", file=sys.stderr)
    sys.exit(1)

import serial


def flash(flash_dir: str, port: str) -> bool:
    flash_args_path = os.path.join(flash_dir, "flash_args")
    if not os.path.exists(flash_args_path):
        print(f"[run_tests] flash_args not found in {flash_dir}", file=sys.stderr)
        return False

    cmd = [
        sys.executable, "-m", "esptool",
        "--chip", "esp32c6",
        "-b", "460800",
        "-p", port,
        "--before", "default-reset",
        "--after", "hard-reset",
        "write-flash",
        "--flash-mode", "dio",
        "--flash-size", "2MB",
        "--flash-freq", "80m",
        "@flash_args",
    ]
    print(f"[run_tests] flashing from {flash_dir} ...", file=sys.stderr)
    result = subprocess.run(cmd, cwd=flash_dir)
    return result.returncode == 0


def read_serial(port: str, baud: int, timeout: float, idle_timeout: float, end_marker: str | None) -> int:
    print(f"[run_tests] opening {port} at {baud} baud, timeout={timeout}s, idle={idle_timeout}s", file=sys.stderr)
    try:
        with serial.Serial(port, baud, timeout=1) as ser:
            ser.setDTR(False)
            ser.setRTS(True)
            time.sleep(0.2)
            ser.setRTS(False)
            ser.setDTR(True)
            time.sleep(0.5)

            deadline = time.time() + timeout
            last_data = time.time()

            while time.time() < deadline:
                raw = ser.readline()
                if raw:
                    last_data = time.time()
                    line = raw.decode("utf-8", errors="replace").rstrip()
                    print(line, flush=True)
                    if end_marker and end_marker in line:
                        break
                elif time.time() - last_data > idle_timeout:
                    print("[run_tests] idle timeout", file=sys.stderr)
                    break

    except serial.SerialException as e:
        print(f"[run_tests] serial error: {e}", file=sys.stderr)
        return 1

    print("[run_tests] done", file=sys.stderr)
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Flash ESP32-C6 test firmware and capture serial output."
    )
    parser.add_argument(
        "--port", default="/dev/ttyACM0",
        help="Serial port (default: /dev/ttyACM0)",
    )
    parser.add_argument(
        "--baud", type=int, default=115200,
        help="Baud rate (default: 115200)",
    )
    parser.add_argument(
        "--flash-dir", default="test_app/build",
        help="Directory containing flash_args (default: test_app/build)",
    )
    parser.add_argument(
        "--timeout", type=float, default=60.0,
        help="Global read deadline in seconds (default: 60)",
    )
    parser.add_argument(
        "--idle-timeout", type=float, default=10.0,
        help="Stop after this many seconds with no data (default: 10)",
    )
    parser.add_argument(
        "--end-marker", default=None, metavar="STRING",
        help="Stop reading when this string appears in a line (e.g. 'Failures' for Unity summary)",
    )
    parser.add_argument(
        "--no-flash", action="store_true",
        help="Skip flashing, only read serial output",
    )
    args = parser.parse_args()

    if not args.no_flash:
        if not flash(args.flash_dir, args.port):
            print("[run_tests] flash failed", file=sys.stderr)
            return 1

    return read_serial(args.port, args.baud, args.timeout, args.idle_timeout, args.end_marker)


if __name__ == "__main__":
    sys.exit(main())
