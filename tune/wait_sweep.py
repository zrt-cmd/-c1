#!/usr/bin/env python3
"""Wait for SWEEP_DONE in /tmp/sweep.out and print the log."""
import time

p = "/tmp/sweep.out"
for _ in range(80):
    try:
        s = open(p).read()
    except FileNotFoundError:
        s = ""
    if "SWEEP_DONE" in s:
        print(s)
        break
    time.sleep(10)
else:
    print("TIMEOUT")
    print(s)
