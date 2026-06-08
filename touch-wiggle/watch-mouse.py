#!/usr/bin/env python3

"""watch-mouse.py - show when the pointer moves, and how long it sat still
before each move. Leave your real mouse alone and watch for a blip roughly
once a minute to confirm the Tomu wiggle is reaching the host.

Run:  ./watch-mouse.py   (or: python3 watch-mouse.py)

Reads the cursor position via macOS CoreGraphics through ctypes, so there are
no dependencies to install and no special permissions needed.
"""

import ctypes
import time
from datetime import datetime

# Load the system frameworks directly; dlopen resolves these from the dyld cache.
cg = ctypes.CDLL("/System/Library/Frameworks/CoreGraphics.framework/CoreGraphics")
cf = ctypes.CDLL("/System/Library/Frameworks/CoreFoundation.framework/CoreFoundation")


class CGPoint(ctypes.Structure):
    _fields_ = [("x", ctypes.c_double), ("y", ctypes.c_double)]


# CGEventRef CGEventCreate(CGEventSourceRef source);
cg.CGEventCreate.restype = ctypes.c_void_p
cg.CGEventCreate.argtypes = [ctypes.c_void_p]
# CGPoint CGEventGetLocation(CGEventRef event);
cg.CGEventGetLocation.restype = CGPoint
cg.CGEventGetLocation.argtypes = [ctypes.c_void_p]
# void CFRelease(CFTypeRef cf);
cf.CFRelease.argtypes = [ctypes.c_void_p]


def mouse_location():
    event = cg.CGEventCreate(None)
    point = cg.CGEventGetLocation(event)
    cf.CFRelease(event)
    return point.x, point.y


POLL_INTERVAL = 0.01  # 10 ms

last_x, last_y = mouse_location()
last_move = time.monotonic()
moves = 0

print("Watching the pointer. Keep your mouse still and wait for the wiggle.")
print("(Ctrl-C to stop)\n")

try:
    while True:
        x, y = mouse_location()
        dx, dy = x - last_x, y - last_y
        if dx or dy:
            idle = time.monotonic() - last_move
            moves += 1
            # CoreGraphics origin is top-left, so +dy already means "down".
            print(f"[{datetime.now():%H:%M:%S}] move #{moves}  "
                  f"d=({dx:+.1f},{dy:+.1f})  after {idle:.1f}s still")
            last_x, last_y = x, y
            last_move = time.monotonic()
        time.sleep(POLL_INTERVAL)
except KeyboardInterrupt:
    print("\nstopped.")
