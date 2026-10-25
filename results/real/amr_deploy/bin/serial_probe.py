#!/usr/bin/env python3
"""Listen on a serial port at several baud rates. Read only, never writes."""
import os, sys, termios, time, select

BAUDS = [('B2000000', termios.B2000000), ('B115200', termios.B115200),
         ('B921600', termios.B921600), ('B57600', termios.B57600)]

def listen(dev, name, speed, secs=3.0):
    try:
        fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    except Exception as e:
        return f"  {name:10s} OPEN FAILED: {e}"
    try:
        a = termios.tcgetattr(fd)
        a[0] = a[1] = a[3] = 0
        a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        a[4] = a[5] = speed
        termios.tcsetattr(fd, termios.TCSANOW, a)
        termios.tcflush(fd, termios.TCIFLUSH)
        buf = b''
        t = time.time()
        while time.time() - t < secs:
            r, _, _ = select.select([fd], [], [], 0.05)
            if r:
                buf += os.read(fd, 4096)
        if not buf:
            return f"  {name:10s} silent"
        return f"  {name:10s} {len(buf)} bytes  {buf[:48].hex()}"
    finally:
        os.close(fd)

for dev in sys.argv[1:]:
    print(f"=== {dev} ===")
    if not os.path.exists(dev):
        print("  missing"); continue
    for name, speed in BAUDS:
        print(listen(dev, name, speed))
