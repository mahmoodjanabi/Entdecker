#!/usr//bin/python

import csv
import serial
import signal
import string
import subprocess
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

if len(sys.argv) < 1:
    print "NO waypoint file supplied"
    sys.exit(1)
else:
    iname = sys.argv[1]
    oname = '/tmp/temp_waypoints.txt'

fi = open(iname, 'rb')
reader = csv.reader(fi, CSVDialect())
rover_lat = 0.0
rover_lng = 0.0
rover_alt = 0.0

curr_lat = 0.0
curr_lng = 0.0
curr_alt = 0.0

for row in reader:
    if row[0] == '#':
        if row[1] == 'UpdatedGPS':
            rover_lat = float(row[2])
            rover_lng = float(row[3])
            rover_alt = float(row[4])

fi.close()

out = subprocess.check_output([ "rostopic", "echo", "-n", "1", "/mavros/global_position/global" ], stderr = subprocess.STDOUT).strip()

a = out.split('\n')

for v in a:
    a1 = v.split(':')

    if a1[0] == 'latitude':
        curr_lat = float(a1[1].strip())
    elif a1[0] == 'longitude':
        curr_lng = float(a1[1].strip())
    elif a1[0] == 'altitude':
        curr_alt = float(a1[1].strip())

if curr_lat == 0.0:
    print "NO current fix:-("
    sys.exit(1)

if rover_lat == 0:
    rover_lat = curr_lat
    rover_lng = curr_lng
    rover_alt = curr_alt

diff_lat = curr_lat - rover_lat
diff_lng = curr_lng - rover_lng
diff_alt = curr_alt - rover_alt

print "diffs: %f, %f, %f" % (diff_lat, diff_lng, diff_alt)

fi = open(iname, 'rb')
reader = csv.reader(fi, CSVDialect())

fo = open(oname, 'wb')
writer = csv.writer(fo, CSVDialect())
writer.writerow(('QGC WPL 120', ))
writer.writerow(('#', 'UpdatedGPS2', rover_lat + diff_lat, rover_lng + diff_lng, rover_alt + diff_alt))

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
                r1 = list(row)
                r1.insert(0, '#')
                # writer.writerow(r1)

                row[8] = float(row[8]) + diff_lat
                row[9] = float(row[9]) + diff_lng

                if float(row[10]) < 1000.0:
                    row[10] = float(row[10]) + diff_alt

            writer.writerow(row)

fi.close()
fo.close()

subprocess.call([ "rosrun", "mavros", "mavcmd", "long", "--", "181", "1.0", "0.0", "0.0", "0.0", "0.0", "0.0", "0.0" ], stderr = subprocess.STDOUT)
subprocess.call([ "rosrun", "mavros", "mavcmd", "long", "--", "181", "0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "0.0" ], stderr = subprocess.STDOUT)
time.sleep(10)
subprocess.call([ "rosrun", "mavros", "mavsys", "-v", "mode", "-c", "MANUAL" ], stderr = subprocess.STDOUT)
# subprocess.call([ "rosrun", "mavros", "mavcmd", "-v", "long", "--", "178", "1", "2.5", "-1", "0", "0", "0", "0" ], stderr = subprocess.STDOUT)
subprocess.call([ "rosrun", "mavros", "mavcmd", "-v", "long", "--", "178", "1", "2.0", "-1", "0", "0", "0", "0" ], stderr = subprocess.STDOUT)
# subprocess.call([ "rosrun", "mavros", "mavparam", "-v", "set", "CRUISE_SPEED", "2.5" ], stderr = subprocess.STDOUT)
subprocess.call([ "rosrun", "mavros", "mavparam", "-v", "set", "CRUISE_SPEED", "2.0" ], stderr = subprocess.STDOUT)
# subprocess.call([ "rosrun", "mavros", "mavparam", "-v", "set", "AUTO_KICKSTART", "2.0" ], stderr = subprocess.STDOUT)
# subprocess.call([ "rosrun", "mavros", "mavparam", "-v", "set", "AUTO_KICKSTART", "3.0" ], stderr = subprocess.STDOUT)
subprocess.call([ "rostopic", "pub", "-1", "/mavros/rc/override", "mavros_msgs/OverrideRCIn", "--", "[ 0, 0, 0, 0, 0, 0, 0, 0 ]" ], stderr = subprocess.STDOUT)
subprocess.call([ "rosrun", "mavros", "mavwp", "-v", "load", oname ], stderr = subprocess.STDOUT)
subprocess.call([ "rosrun", "mavros", "mavwp", "-v", "setcur", "0" ], stderr = subprocess.STDOUT)
subprocess.call([ "rosrun", "mavros", "mavsys", "-v", "mode", "-c", "AUTO" ], stderr = subprocess.STDOUT)
subprocess.call([ "rosrun", "mavros", "mavcmd", "long", "--", "181", "1.0", "0.0", "0.0", "0.0", "0.0", "0.0", "0.0" ], stderr = subprocess.STDOUT)
subprocess.call([ "rosrun", "mavros", "mavcmd", "long", "--", "181", "0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "0.0" ], stderr = subprocess.STDOUT)
