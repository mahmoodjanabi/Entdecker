#!/bin/bash -x

curl -o /tmp/base_solution.sh https://www.bytesalad.org/~marco/base_solution.sh

. /tmp/base_solution.sh
rm -f /tmp/base_solution.sh

sed \
    -e "s/@BASE_LATITUDE@/$base_latitude/" \
    -e "s/@BASE_LONGITUDE@/$base_longitude/" \
    -e "s/@BASE_HEIGHT@/$base_height/" \
    < /disk1/marcow/DGPS/rtklib.git/RTKLIB/app/rtkrcv/gcc/rtkrcv.conf.template > /disk1/marcow/DGPS/rtklib.git/RTKLIB/app/rtkrcv/gcc/rtkrcv.conf

(cd /disk1/marcow/DGPS/rtklib.git/RTKLIB/app/rtkrcv/gcc/ && nohup ./rtkrcv -s -p 11234 -o rtkrcv.conf &)

sleep 30

nohup /disk1/marcow/APM/gps_handler/status_display.py &



