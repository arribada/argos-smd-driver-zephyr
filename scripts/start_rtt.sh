#!/bin/bash
# Start RTT logging - kills existing JLink instances and starts fresh

# Kill existing JLink processes
pkill -9 -f JLink 2>/dev/null || true
sleep 1

# Start JLink GDB Server in background
JLinkGDBServerCLExe -device NRF52840_XXAA -if SWD -speed 4000 -RTTTelnetPort 19021 -nogui -silent &
JLINK_PID=$!

# Wait for server to start
sleep 2

# Start RTT log viewer
python3 "$(dirname "$0")/rtt_logs.py"

# Cleanup
kill $JLINK_PID 2>/dev/null
