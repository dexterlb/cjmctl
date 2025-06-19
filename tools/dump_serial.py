#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.12"
# dependencies = [
# "pyserial",
# ]
# ///

import time
import sys
import os
import traceback

import serial

def do_the_dump(port, baud):
    ser = serial.Serial(port, baud)
    while True:
        c = ser.read()
        sys.stdout.buffer.write(c)
        sys.stdout.flush()

if __name__ == '__main__':
    if len(sys.argv) != 3:
        sys.stderr.write(f'usage: {sys.argv[0]} <port> <baud>\n')
        sys.stderr.write(f'will dump everything that comes on the serial port to stdout\n')
        sys.exit(1)

    do_the_dump(sys.argv[1], int(sys.argv[2]))
