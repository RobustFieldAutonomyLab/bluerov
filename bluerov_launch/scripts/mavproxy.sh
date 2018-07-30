#!/usr/bin/env bash
python /usr/local/bin/mavproxy.py --master=udp:192.168.2.1:14550 --out=udp:192.168.2.1:14551 --out=udp:192.168.2.1:14552 --out=udp:192.168.2.1:14553 --speech