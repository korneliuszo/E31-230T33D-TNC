#!/usr/bin/env python3

import xmltodict

d =  xmltodict.parse(open("axradiolabstate.xml").read())

reg_common = []
reg_rx = []
reg_tx = []


for r in d["AXRadioLabState"]["REGISTERS"]["REGISTER"]:
    addr = int(r['@addr'],16)
    t = (addr&0xFFF, int(r['@value'],16),r["@name"])
    if addr & 0x3000 == 0x0000:
        reg_common.append(t)
    if addr & 0x3000 == 0x1000:
        reg_tx.append(t)
    if addr & 0x3000 == 0x2000:
        reg_rx.append(t)

f=open("SDCC/config_values.c","w")

header="""//AUTOGENERATED

#include "../config_values.h"

"""
f.write(header)

for reg,name in ((reg_common,"config_common"),
                 (reg_rx,"config_rx"),
                 (reg_tx,"config_tx")):
    f.write(f"register_pair {name}[]={{\n")
    for addr,val,name in reg:
        f.write(f"\t{{0x{addr:03X}, 0x{val:02X} }}, //{name}\n")
    f.write("\t{0x000, 0x00 }, //TERMINATOR\n")
    f.write("};\n\n")