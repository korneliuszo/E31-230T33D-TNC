#!/usr/bin/env python3

import sys
import struct
import re

regs_re = re.compile(r"#define\s*(.*?REG.*?)\s\s*(.*?)\s")
regs_pattern_re = re.compile(r"#define\s*(.*?)\s\s*(.*?)\s")
regs = {}
regs_pattern = {}
with open("../ax_reg.h","r") as regs_file:
    for line in regs_file:
        match = regs_re.match(line)
        if match:
            regs[int(match[2],16)] = match[1]
        else:
            match = regs_pattern_re.match(line)
            if match:
                regs_pattern[int(match[2],16)] = match[1]          

regs_merge = {}
for k,v in regs.items():
    if v.find("AX_REG_RX_PARAMETER")!=-1:
        for k2,v2 in regs_pattern.items():
            regs_merge[k+k2] = v + "+" + v2

for k,v in regs_merge.items():
    regs[k] = v

f=open(sys.argv[1],"rb")

type = f.read(1)

while type:
    if type[0] == 0xff:
        print("Startup glitch?")
    elif type[0] == 0x01:
        s=struct.unpack(">HB",f.read(3))
        print(f"U8 Read: 0x{s[0]:04X}, 0x{s[1]:02X} REG: {regs.get(s[0],'UNKNOWN')}")
    elif type[0] == 0x02:
        s=struct.unpack(">HH",f.read(4))
        print(f"U16 Read: 0x{s[0]:04X}, 0x{s[1]:02X} REG: {regs.get(s[0],'UNKNOWN')}")
    elif type[0] == 0x03:
        s=struct.unpack(">HB",f.read(3))
        print(f"U8 Write: 0x{s[0]:04X}, 0x{s[1]:02X} REG: {regs.get(s[0],'UNKNOWN')}")
    elif type[0] == 0x04:
        s=struct.unpack(">HH",f.read(4))
        print(f"U16 Write: 0x{s[0]:04X}, 0x{s[1]:02X} REG: {regs.get(s[0],'UNKNOWN')}")
    elif type[0] == 0x05:
        s=struct.unpack(">HI",f.read(6))
        print(f"U24 Write: 0x{s[0]:04X}, 0x{s[1]:02X} REG: {regs.get(s[0],'UNKNOWN')}")
    elif type[0] == 0x06:
        s=struct.unpack(">HI",f.read(6))
        print(f"U32 Write: 0x{s[0]:04X}, 0x{s[1]:02X} REG: {regs.get(s[0],'UNKNOWN')}")
    elif type[0] == 0x07:
        print("Reread value differ!!!")
    elif type[0] == 0x08:
        s=struct.unpack(">B",f.read(1))
        print(f"RADIOSTATE = 0x{s[0]:02X}")
    elif type[0] == 0x00:
        packet = b""
        while True:
            byte = f.read(1)
            if byte[0] == 0xC0:
                break
            packet+=byte
        print("RX: ",packet)
    type = f.read(1)
