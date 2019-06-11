# BlueROV2 ROS Package 

TODO

# Installation

## Dependencies

```
sudo apt-get install socat
sudo pip install mavproxy pymavlink
```

## QGroundControl

Before installing QGroundControl for the first time:

```
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav -y
```

Logout and login again to enable the change to user permissions.

To install QGroundControl for Ubuntu Linux 16.04 LTS or later, Download [QGroundControl.AppImage](https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage)

```
chmod +x ./QGroundControl.AppImage
sudo mv ./QGroundControl.AppImage /usr/local/bin/QGroundControl.AppImage
```

# Configuration

## IP addresses

Computer: `192.168.2.1`
BlueROV2: `192.168.2.2` user:password: pi:companion
Oculus M750d: `192.168.2.3`
Oculus M120d: `192.168.2.4`

## Companion setup

ssh to `pi@192.169.2.2` and add the following lines to `companion/.companion.rc` to enable virtual serial ports over TCP.

```
socat tcp-listen:14661,reuseaddr,fork,ignoreeof file:/dev/ttyAMA0,nonblock,waitlock=/var/run/ttyAMA0.lock,b115200,raw,echo=0 &
socat tcp-listen:14660,reuseaddr,fork file:/dev/rowe_dvl,nonblock,waitlock=/var/run/rowe_dvl.lock,b115200,raw,echo=0 &
socat -u udp4-recvfrom:25102,fork,reuseaddr udp4-sendto:192.168.2.1:14662 &
```

## Disable camera

ssh to `pi@192.168.2.2` and comment the following line in `companion/.companion.rc`.

```
sudo -H -u pi screen -dm -S video $COMPANION_DIR/tools/streamer.py
```
