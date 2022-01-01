#!/usr/bin/env python3

import sys
import serial

s=serial.Serial(sys.argv[1],9600,8,"N",1)

s.timeout = 1

s.write(b"\xC0\xC0\x07Hello\xC0")
print(s.read(7))