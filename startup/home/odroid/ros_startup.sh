#!/bin/bash

source /opt/ros/indigo/setup.bash
source $HOME/catkin_ws/devel/setup.bash
unset GTK_IM_MODULE

# ethtool eth0 2>&1 | fgrep -q 'Link detected: yes'
# if [ $? -ne 0 ]
# then
    # export ROS_IP=192.168.8.1
# fi

date >> nohup.out
env | fgrep ROS >> nohup.out

nohup roscore &

sleep 10

nohup roslaunch mw_mavros mw1.launch &

sleep 40

nohup roslaunch mw_mavros mw2.launch &
rosrun mavros mavparam set SYSID_MYGCS 1
rostopic pub -1 /mw/search std_msgs/Bool -- false

gps_serial=$(basename $(readlink -f /dev/serial/by-path/platform-s5p-ehci-usb-0\:3.3.4\:1.0-port0))

rm base_solution.sh
wget 'https://www.bytesalad.org/~marco/base_solution.sh'

. ./base_solution.sh

sed \
    -e "s/@BASE_LATITUDE@/$base_latitude/" \
    -e "s/@BASE_LONGITUDE@/$base_longitude/" \
    -e "s/@BASE_HEIGHT@/$base_height/" \
    -e "s/@OUT_SERIAL@/$gps_serial/" \
    < /home/odroid/catkin_ws/src/mw/mw_rtkrcv/config/rtkrcv.conf.template > /home/odroid/catkin_ws/src/mw/mw_rtkrcv/config/rtkrcv.conf

roslaunch mw_rtkrcv gps.launch &

echo "============= ros_startup.sh end ===============" >> nohup.out
date >> nohup.out

