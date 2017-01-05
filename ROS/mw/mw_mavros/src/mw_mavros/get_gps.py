#!/usr//bin/python

import subprocess

out = subprocess.check_output([ "rostopic", "echo", "-n", "1", "/mavros/global_position/global" ], stderr = subprocess.STDOUT).strip()

a = out.split('\n')

for v in a:
    a1 = v.split(':')

    if a1[0] == 'latitude':
        lat = a1[1].strip()
    elif a1[0] == 'longitude':
        lng = a1[1].strip()
    elif a1[0] == 'altitude':
        alt = a1[1].strip()

print "#\tRoverGPS\t%s\t%s\t%s" % (lat, lng, alt)
