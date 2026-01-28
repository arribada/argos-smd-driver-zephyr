#!/bin/bash
# RTT Logs viewer - simple and robust

# Kill existing JLink
pkill -9 -f JLink 2>/dev/null
sleep 1

# Flash if requested
if [ "$1" = "flash" ]; then
    echo "=== Flashing... ==="
    west flash
    sleep 1
fi

# Start GDB server in background
echo "=== Starting JLink GDB Server ==="
JLinkGDBServerCLExe -device NRF52840_XXAA -if SWD -speed 4000 -RTTTelnetPort 19021 -nogui -silent &
JLINK_PID=$!
sleep 3

# Connect to RTT
echo "=== Connecting to RTT ==="
exec 3<>/dev/tcp/localhost/19021
echo "=== RTT Logs ==="

# Read and display
while IFS= read -r -n1 char <&3; do
    printf '%s' "$char"
done

# Cleanup
kill $JLINK_PID 2>/dev/null
