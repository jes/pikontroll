#!/usr/bin/env python

import serial

ser = serial.Serial("/dev/ttyUSB0", 9600, timeout=1)

ser.write("target 0 1000\n")
ser.write("read 0\n")

while True:
    print ser.readline()
