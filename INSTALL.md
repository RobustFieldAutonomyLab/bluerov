# Installation

## Dependencies

```
sudo apt-get install socat

#sudo apt-get install python-pip
sudo pip install catkin_tools mavproxy pymavlink
```

## Build workspace

```
$ mkdir -p WORKSPACE/src
$ cd WORKSPACE
$ catkin init
$ catkin config --merge-devel
$ catkin config --cmake-args -DCMAKE_BUILD_TYPE=Releas
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
