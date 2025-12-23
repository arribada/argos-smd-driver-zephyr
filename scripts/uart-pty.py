#!/usr/bin/env python3
"""
Wait for a /dev/ttyACM* device to appear (max 10 s), then forward its
output to stdout.  Works as a --device-serial-pty helper for Twister.
*/
"""

import os
import sys
import time
import serial
import glob

BAUD = 115200
TIMEOUT_WAIT = 10.0        
TIMEOUT_SERIAL = 0.1
WAIT_BEFORE_SEARCH = 5

def find_ttyACM(timeout: float) -> str:
    """Return first /dev/ttyACM* that appears within <timeout> seconds."""
    end = time.time() + timeout
    while time.time() < end:
        candidates = glob.glob('/dev/ttyACM*')
        if candidates:
            return sorted(candidates)[0]
        time.sleep(0.2)
    raise RuntimeError('No /dev/ttyACM* device found within timeout')

def main() -> None:
    try:
        time.sleep(WAIT_BEFORE_SEARCH)
        port = find_ttyACM(TIMEOUT_WAIT)
    except RuntimeError as e:
        print(str(e), file=sys.stderr)
        sys.exit(1)

    try:
        with serial.Serial(port, BAUD, timeout=TIMEOUT_SERIAL) as ser:
            while True:
                data = ser.read(4096)
                if data:
                    sys.stdout.buffer.write(data)
                    sys.stdout.flush()
    except serial.SerialException as e:
        print(f'Serial error: {e}', file=sys.stderr)
        sys.exit(2)
    except KeyboardInterrupt:
        print('\nInterrupted by user', file=sys.stderr)

if __name__ == '__main__':
    main()
