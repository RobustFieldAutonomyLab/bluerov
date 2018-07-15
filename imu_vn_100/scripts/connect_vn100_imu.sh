#!/usr/bin/env bash
if [ ! -a /dev/vn100_imu ] ; then
    sudo socat pty,link=/dev/vn100_imu,waitslave,raw,echo=0,ignoreeof tcp:192.168.2.2:14661 &
    while true ; do
        if [ -a /dev/vn100_imu ]; then
            sudo chmod 777 /dev/vn100_imu
            break
        else
            sleep 1
        fi
    done
fi
