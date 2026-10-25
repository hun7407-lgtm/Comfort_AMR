#!/usr/bin/env python3
"""Send the control board's NOP query to every serial port at every plausible
baud, and report which one answers. Sends only NOP -- the robot cannot move."""
import os, sys, termios, time, select, glob

def crc16(d):
    c = 0
    for b in d:
        c ^= b << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) if c & 0x8000 else c << 1
        c &= 0xffff
    return c

_idx = 0
def pkt(cmd):
    global _idx
    _idx = (_idx + 1) & 0xff
    h = bytes([1, 0, len(cmd), _idx, 2]) + cmd.encode()
    c = crc16(h)
    return b'\x01' + h + b'\x03' + bytes([c & 0xff, c >> 8, 4])

BAUDS = [('2000000', termios.B2000000), ('921600', termios.B921600),
         ('576000', termios.B576000), ('460800', termios.B460800),
         ('230400', termios.B230400), ('115200', termios.B115200),
         ('57600', termios.B57600), ('38400', termios.B38400),
         ('19200', termios.B19200), ('9600', termios.B9600)]

def probe(dev, name, speed, tries=4):
    try:
        fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    except Exception as e:
        return f"open failed: {e}"
    try:
        a = termios.tcgetattr(fd)
        a[0] = a[1] = a[3] = 0
        a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        a[4] = a[5] = speed
        termios.tcsetattr(fd, termios.TCSANOW, a)
        got = b''
        for _ in range(tries):
            termios.tcflush(fd, termios.TCIFLUSH)
            os.write(fd, pkt('REPTYPE=BUWP;NOP'))
            t = time.time()
            while time.time() - t < 0.20:
                r, _, _ = select.select([fd], [], [], 0.02)
                if r:
                    got += os.read(fd, 4096)
            if got:
                break
        if not got:
            return None
        return f"{len(got)} bytes: {got[:60].hex()}"
    finally:
        os.close(fd)

devs = sys.argv[1:] or sorted(set(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
                                  + glob.glob('/dev/ttyCH341*')))
print("ports:", devs)
hit = False
for dev in devs:
    for name, speed in BAUDS:
        r = probe(dev, name, speed)
        if r:
            print(f"  *** {dev} @ {name}  ->  {r}")
            hit = True
if not hit:
    print("  no port answered at any baud")
