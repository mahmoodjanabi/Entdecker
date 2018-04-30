#!/bin/bash

source /opt/ros/indigo/setup.bash
source $HOME/catkin_ws/devel/setup.bash
unset GTK_IM_MODULE

ethtool eth0 2>&1 | fgrep -q 'Link detected: yes'
if [ $? -eq 0 ]
then
    export ROS_IP=$(ifconfig eth0 | awk -F: '$1 ~ /inet addr/ && $2 !~ /^127\./ && $2 !~ /^169\./ && $2 !~ /^172\./ { gsub(/ .*/, "", $2); print $2; }')
else
    export ROS_IP=$(ifconfig wlan1 | awk -F: '$1 ~ /inet addr/ && $2 !~ /^127\./ && $2 !~ /^169\./ && $2 !~ /^172\./ { gsub(/ .*/, "", $2); print $2; }')
fi

export ROS_MASTER_URI="http://$ROS_IP:11311"

date >> nohup.out
env | fgrep ROS >> nohup.out

nohup roscore &

sleep 10

nohup roslaunch mw_mavros mw1.launch &

sleep 40

if [ 'display' == "$1" ]
then
    nohup roslaunch mw_mavros mw2.launch debug:=true &

    rostopic pub -1 /mw/search std_msgs/Bool -- true
    sleep 10

    (export DISPLAY=:0.0 && rosrun image_view image_view image:=/mw/image_topic) &
    sleep 10

    (export DISPLAY=:0.0 && /home/odroid/bin/totem_restarter.py) &
    sleep 10

    exit 0
fi

nohup rosbag record -o /sd/odroid/rosbag_$(date +'%Y%m%d%H%M%S').bag -e '/((camera)|(mavros)|(mw))/.*' /scan &

nohup roslaunch mw_mavros mw2.launch &
rosrun mavros mavparam set SYSID_MYGCS 1
nohup roslaunch rplidar_ros rplidar.launch &
rostopic pub -1 /mw/search std_msgs/Bool -- false
sleep 10

# UBlox USB ACM0 /dev/serial/by-path/platform-s5p-ehci-usb-0\:3.2.1\:1.0
# USB -> TTL Serial cp210x
gps_serial=$(basename $(readlink -f /dev/serial/by-path/platform-s5p-ehci-usb-0\:3.2.4\:1.0-port0))
gps_serial=gps-out

rm -f base_solution.sh
wget --timeout=10 'https://www.bytesalad.org/~marco/base_solution.sh'

if [ $? -eq 0 -a -f base_solution.sh ]
then
    . ./base_solution.sh

    sed \
        -e "s/@BASE_LATITUDE@/$base_latitude/" \
        -e "s/@BASE_LONGITUDE@/$base_longitude/" \
        -e "s/@BASE_HEIGHT@/$base_height/" \
        -e "s/@OUT_SERIAL@/$gps_serial/" \
        < $HOME/catkin_ws/src/mw/mw_rtkrcv/config/rtkrcv.conf.template \
        > $HOME/catkin_ws/src/mw/mw_rtkrcv/config/rtkrcv.conf

    nohup roslaunch mw_rtkrcv gps.launch &
fi

echo "============= ros_startup.sh end ===============" >> nohup.out
date >> nohup.out

