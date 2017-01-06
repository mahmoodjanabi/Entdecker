#!/usr/bin/python

import csv
import math
import serial
import signal
import string
import sys
import termios
import threading
import time
import tty

class CSVDialect(csv.Dialect):
    delimiter = '\t'
    doublequote = False
    skipinitialspace = True
    lineterminator = '\r\n'
    quoting = csv.QUOTE_NONE

if len(sys.argv) < 2:
    sys.exit(1)
else:
    iname = sys.argv[1]
    oname = sys.argv[2]

fi = open(iname, 'rb')
reader = csv.reader(fi, CSVDialect())
rover_lat = 0.0
rover_lng = 0.0
rover_alt = 0.0

laptop_lat = 0.0
laptop_lng = 0.0
laptop_alt = 0.0

for row in reader:
    if row[0] == '#':
        if row[1] == 'RoverGPS':
            rover_lat = float(row[2])
            rover_lng = float(row[3])
            rover_alt = float(row[4])
        elif row[1] == 'BasePoint':
            laptop_lat = float(row[2])
            laptop_lng = float(row[3])
            laptop_alt = float(row[4])

fi.close()

if rover_lat == 0.0 or laptop_lat == 0.0:
    sys.exit(1)

diff_lat = rover_lat - laptop_lat
diff_lng = rover_lng - laptop_lng
diff_alt = rover_alt - laptop_alt

fi = open(iname, 'rb')
reader = csv.reader(fi, CSVDialect())

fo = open(oname, 'wb')
writer = csv.writer(fo, CSVDialect())
writer.writerow(('QGC WPL 120', ))
writer.writerow(('#', 'UpdatedGPS', laptop_lat + diff_lat, laptop_lng + diff_lng, laptop_alt + diff_alt))

for row in reader:
    if row[0] == '#':
        # writer.writerow(row)
        dummy = 0
    elif row[0].isdigit():
        i = int(row[0])

        if i == 0:
            writer.writerow(row)
        else:
            cmd = int(row[3])

            if cmd == 16 or cmd == 21:
                # r1 = list(row)
                # writer.writerow(r1)

                row[8] = float(row[8]) + diff_lat
                row[9] = float(row[9]) + diff_lng

                if float(row[10]) < 1000.0:
                    row[10] = float(row[10]) + diff_alt

            writer.writerow(row)

fi.close()
fo.close()
