#!/usr/bin/python2

import dbus
from datetime import datetime
import glob
import subprocess
import time
from pymediainfo import MediaInfo

def video_duration(list):
    duration = 0
    for video in list:
        media_info = MediaInfo.parse(video)
        for track in media_info.tracks:
            if track.track_type == 'Video':
                duration += track.duration / 1000

    return duration


videos = '/home/odroid/Videos/*'
pause = 5 * 60

cmd = [ '/usr/bin/totem', '--fullscreen', '--display=:0.0' ]
v = glob.glob(videos)
v.sort()
duration = video_duration(v)

# print "v= %s" % v
c = cmd + v

print "cmd= %s  duraction= %f + %d" % (c, duration, pause)
totem_proc = subprocess.Popen(c, env = { 'DISPLAY' : ':0.0' })
start_time = datetime.now()
time.sleep(1)

session_bus = dbus.SessionBus()

while True:
    status = None
    totem_o = None

    try:
        totem_o = session_bus.get_object('org.mpris.MediaPlayer2.totem', '/org/mpris/MediaPlayer2')
        totem_props = dbus.Interface(totem_o, dbus_interface = dbus.PROPERTIES_IFACE)
        status = totem_props.GetAll('org.mpris.MediaPlayer2.Player')['PlaybackStatus']
    except dbus.exceptions.DBusException:
        print "Not running?"

    # print "status= %s" % status

    if status == 'Stopped' or status == 'Paused':
        # print "restart"
        totem_player = dbus.Interface(totem_o, dbus_interface= 'org.mpris.MediaPlayer2')

        try:
            totem_player.Quit()
        except dbus.exceptions.DBusException:
            dummy = 0

        time.sleep(1)
        status = None

    if status == None and (datetime.now() - start_time).total_seconds() > duration + pause:
        v = glob.glob(videos)
        v.sort()
        duration = video_duration(v)
        c = cmd + v
        totem_proc = subprocess.Popen(c)
        start_time = datetime.now()
        print "cmd= %s  duraction= %f + %d" % (c, duration, pause)
        time.sleep(2)

    time.sleep(10)
