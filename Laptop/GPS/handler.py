#!/usr/bin/python

import csv
import math
import serial
import signal
import string
import subprocess
import sys
import termios
import threading
import time
import tty

class Getch:
    def __call__(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

class CSVDialect(csv.Dialect):
    delimiter = '\t'
    doublequote = False
    skipinitialspace = True
    lineterminator = '\r\n'
    quoting = csv.QUOTE_NONE

class GpsValue:
    lock = threading.Lock()
    lat = 0.0
    lng = 0.0
    alt = 0.0

class GpsReader(threading.Thread):
    ser = None
    start_called = False
    gps_value = None

    def __init__(self, group = None, target = None, name = None,
                 args = (), kwargs = None, verbose = None):
        threading.Thread.__init__(self, group=group, target = target, name = name,
                                  verbose = verbose)
        self.args = args
        self.kwargs = kwargs
        self.gps_value = args[0]

        return

    def start(self):
        if self.start_called:
            raise RunTimeError

        self.start_called = True
        self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout = 1)

        self.ser.write('$PMTK251,57600*2C\r\n') # 57600 baud
        self.ser.close()

        self.ser.baudrate = 57600
        self.ser.open()

        i = 0                              # Just read some lines
        line = self.ser.readline()
        while line != None and i < 10:
            line = self.ser.readline()
            i += 1

        self.ser.write('$PMTK220,200*2C\r\n')   # send rate 5Hz
        self.ser.write('$PMTK300,200,0,0,0,0*2F\r\n')  # fix update rate 5Hz
        self.ser.write('$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n') # Send all data

        super(GpsReader, self).start()

    def run(self):
        line = self.ser.readline()
        while line != None:
            a = line.strip().split(',')
            if a[0] == '$GPGGA':
                lat_read = 0.0
                long_read = 0.0
                alt_read = 0.0

                if int(a[6]) == 0:                   # invalid
                    valid = int(a[6])
                    # print "invalid? %s\r" % valid
                elif float(a[8]) > 3:                # large hdop:-(
                    hdop_read = float(a[8])
                    # print "hdop: %f\r" % hdop_read
                else:
                    lat_read = float(a[2])           # assume N
                    long_read = float(a[4])          # assume W
                    alt_read = float(a[9])           # ??

                if lat_read != 0.0:
                    lat_real = int(lat_read / 100) + ((5 * ((lat_read * 10000) % 1000000) / 3) / 1000000)
                    long_real = 0 - (int(long_read / 100) + ((5 * ((long_read * 10000) % 1000000) / 3) / 1000000))
                    alt_real = alt_read
                    # print "lat= %f, long= %f\r" % (lat_real, long_real)
                else:
                    lat_real = 0.0
                    long_real = 0.0
                    alt_real = 0.0
                    # print "no fix\r"

                if self.gps_value != None:
                    self.gps_value.lock.acquire()
                    try:
                        self.gps_value.lat = lat_real
                        self.gps_value.lng = long_real
                        self.gps_value.alt = alt_real
                        # print "gps %f %f %f\r" % (self.gps_value.lat, self.gps_value.lng, self.gps_value.alt)
                    finally:
                        self.gps_value.lock.release()

            line = self.ser.readline()
            time.sleep(0)

def calc_diff(lat1, lng1, lat2, lng2):
    lat1_rad = math.radians(lat1)
    lng1_rad = math.radians(lng1)
    lat2_rad = math.radians(lat2)
    lng2_rad = math.radians(lng2)
    dlat_rad = math.radians(lat2 - lat1)
    dlng_rad = math.radians(lng2 - lng1)
    R = 6371000

    a = math.sin(dlat_rad / 2) * math.sin(dlat_rad / 2) + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlng_rad / 2) * math.sin(dlng_rad / 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    d = R * c

    y = math.sin(dlng_rad) * math.cos(lat2_rad)
    x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlng_rad)

    brng = math.degrees(math.atan2(y, x))

    if brng < 0:
        brng += 360

    return (d, brng)

value = GpsValue()
t = GpsReader(args = (value, ))
t.setDaemon(True)
t.start()

if len(sys.argv) > 1:
    fname = sys.argv[1]
else:
    fname = '/tmp/waypoints.txt'

f = open(fname, 'wb')
writer = csv.writer(f, CSVDialect())
writer.writerow(('QGC WPL 120', ))
sequence = 0
writer.writerow((sequence, 1, 0, 16, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1))
sequence += 1
WAYPOINT_RADIUS = 2.0
ROVER_HOST_LIST = [ '192.168.8.1', '192.168.1.16' ]

for h in ROVER_HOST_LIST:
    ping_ret = subprocess.call([ "ping", "-c", "1", "-t", "1", "-q", h ])
    if ping_ret == 0:
       rover_host = h
       break

print "c[lrs] - search & avoid Left/Right/Stop, l - land, r\d\d - relay x -> y, s\d\d - speed x.y m/s, w - waypoint, q - quit"

getch = Getch()
c = getch().lower()
while c != 'q' and ord(c) != 3:
    value.lock.acquire()
    try:
        my_lat = value.lat
        my_long = value.lng
        my_alt = value.alt
        # print "gps received %f %f %f" % (value.lat, value.lng, value.alt)
    finally:
        value.lock.release()

    if my_lat == 0.0:
        print "No fix:-("
    elif c == 'b':                             # base point
        writer.writerow(("#", "BasePoint", my_lat, my_long, my_alt))
        print "b - %f %f %f" % (my_lat, my_long, my_alt)
        if ping_ret == 0:
            r = subprocess.check_output([ "ssh", "-i", "/export/home/marcow/.ssh/id_daheim", "odroid@%s" % rover_host, 'source /opt/ros/indigo/setup.bash && source $HOME/catkin_ws/install/setup.bash && source $HOME/catkin_ws/devel/setup.bash && /home/odroid/catkin_ws/src/mw/mw_mavros/src/mw_mavros/get_gps.py' ])
            ra = r.strip().split('\t')
            writer.writerow(ra)
            l = calc_diff(my_lat, my_long, float(ra[2]), float(ra[3]))
            writer.writerow(("#", "Diff", l[0], l[1]))
            print "  diff %f  %f" % (l[0], l[1])
    elif c == 'w':                             # waypoint
        writer.writerow((sequence, 0, 0, 16, 0.0, WAYPOINT_RADIUS, 0.0, 0.0, my_lat, my_long, my_alt, 1))
        sequence += 1
        print "w - %f %f %f" % (my_lat, my_long, my_alt)
    elif c == 'c':                             # searchpoint
        c1 = getch().lower()
        if c1 == 'l':                          # avoid left
            writer.writerow((sequence, 0, 0, 16, 0.0, WAYPOINT_RADIUS, 0.0, 0.0, my_lat, my_long, 1010.0, 1))
            sequence += 1
            print 'c - left %f %f 1010' % (my_lat, my_long)
        elif c1 == 'r':                        # avoid right
            writer.writerow((sequence, 0, 0, 16, 0.0, WAYPOINT_RADIUS, 0.0, 0.0, my_lat, my_long, 1020.0, 1))
            sequence += 1
            print 'c - right %f %f 1020' % (my_lat, my_long)
        elif c1 == 's':                        # stop afterwards
            writer.writerow((sequence, 0, 0, 16, 0.0, WAYPOINT_RADIUS, 0.0, 0.0, my_lat, my_long, 1030.0, 1))
            sequence += 1
            print 'c - land %f %f 1030' % (my_lat, my_long)
        else:
            print 'c - unknown %s' % c1
    elif c == 's':                             # speed change
        try:
            c1 = float(int(getch()))
            c2 = float(int(getch())) / 10.0
            s = c1 + c2
            writer.writerow((sequence, 0, 0, 178, 0.0, s, -1.0, 0.0, 0.0, 0.0, 0.0, 1))
            sequence += 1
            writer.writerow((sequence, 0, 0, 178, 1.0, s, -1.0, 0.0, 0.0, 0.0, 0.0, 1))
            sequence += 1
            print 's - %f' % s
        except ValueError:
            print "s - speed not understood"
    elif c == 'r':                             # relay
        try:
            r = float(int(getch()))
            v = float(int(getch()))
            writer.writerow((sequence, 0, 0, 181, r, v, 0.0, 0.0, 0.0, 0.0, 0.0, 1))
            sequence += 1
            print "r - %d => %d" % (r, v)
        except ValueError:
            print "r - relay not understood"
    elif c == 'l':                             # land
        writer.writerow((sequence, 0, 0, 181, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1))
        sequence += 1
        writer.writerow((sequence, 0, 0, 181, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1))
        sequence += 1
        writer.writerow((sequence, 0, 0, 21, 0.0, 0.0, 0.0, 0.0, my_lat, my_long, my_alt, 1))
        sequence += 1
        print 'l - %f %f %f' % (my_lat, my_long, my_alt)

    c = getch().lower()

f.close()