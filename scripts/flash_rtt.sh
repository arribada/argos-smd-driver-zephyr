#!/bin/bash
pkill -9 -f JLink 2>/dev/null
sleep 1
west flash && west rtt
