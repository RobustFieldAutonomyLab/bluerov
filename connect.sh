screen -dm -S mavproxy python /usr/local/bin/mavproxy.py --master=udp:192.168.2.1:14550 --out=udp:192.168.2.1:14551 --out=udp:192.168.2.1:14552
screen -dm -S qgc /usr/local/bin/QGroundControl.AppImage
screen -dm -S rti_dvl socat pty,link=/tmp/rti_dvl,waitslave,raw,echo=0,ignoreeof tcp:192.168.2.2:14660
screen -dm -S vn100_imu socat pty,link=/tmp/vn100_imu,waitslave,raw,echo=0,ignoreeof tcp:192.168.2.2:14661
