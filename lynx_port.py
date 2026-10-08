#!/usr/bin/python3
# ================================================================
# lynx_port.py
# Auto-detect or select serial port for LYNX rover.
#
# Usage in any LYNX script:
#   from lynx_port import get_port
#   ser = serial.Serial(port=get_port(), baudrate=115200, timeout=0.05)
# ================================================================

import serial.tools.list_ports
import sys


def get_port():
    """Find and return the serial port for the LYNX rover."""
    ports = list(serial.tools.list_ports.comports())

    if not ports:
        print("ERROR: No serial ports found.")
        sys.exit(1)

    # Filter likely candidates
    candidates = []
    for p in ports:
        name = p.device.lower()
        if 'ttyusb' in name or 'ttyacm' in name or 'com' in name:
            candidates.append(p)

    if not candidates:
        candidates = ports

    # One port — use it
    if len(candidates) == 1:
        print(f"  Port: {candidates[0].device}")
        return candidates[0].device

    # Multiple — let user pick
    print("  Available ports:")
    for i, p in enumerate(candidates):
        desc = p.description or ""
        print(f"    {i+1}. {p.device}  {desc}")

    choice = input("  Select option: ").strip()
    try:
        idx = int(choice) - 1
        if 0 <= idx < len(candidates):
            return candidates[idx].device
    except:
        pass

    print(f"  Invalid choice, using {candidates[0].device}")
    return candidates[0].device
