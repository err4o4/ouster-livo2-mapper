kill all ros processes 
pkill -f ros

export DISABLE_ROS1_EOL_WARNINGS=1

Use calib data
https://github.com/hku-mars/FAST_LIO/issues/337

Set lidar address
```
https://github.com/KumarRobotics/kr_autonomous_flight/wiki/New-Robot:-Ouster-OS-1---Setup---PTP#set-up-static-ip
```

Set PTP Sync 
```
sudo ptp4l -i eth0 -m -4
```

Build container
```
docker build -t fast-livo2:noetic-r35.4.1 .
```

Start container
```
// Mac OS
docker run -it --rm \
  -p 8765:8765 \
  --name fast-livo2 \
  -v ./ros_overlay/FAST-Calib/config:/opt/catkin_ws/src/FAST-Calib/config \
  -v ./ros_overlay/FAST-Calib/calib_data:/opt/catkin_ws/src/FAST-Calib/calib_data \
  -v ./ros_overlay/FAST-Calib/output:/opt/catkin_ws/src/FAST-Calib/output \
  -v ./ros_overlay/FAST-LIVO2/config:/opt/catkin_ws/src/FAST-LIVO2/config \
  -v ./scripts:/opt/scripts \
  --name fast-livo2 \
  fast-livo2:noetic-r35.4.1


// Jetson
export DISPLAY=:1
xhost +local:root  
docker run -it --rm \
  --runtime nvidia \
  --privileged \
  --network host \
  -p 8765:8765 \
  -e DISPLAY=$DISPLAY \
  -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
  -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /dev:/dev \
  -v ./ros_overlay/FAST-Calib/config:/opt/catkin_ws/src/FAST-Calib/config \
  -v ./ros_overlay/FAST-Calib/calib_data:/opt/catkin_ws/src/FAST-Calib/calib_data \
  -v ./ros_overlay/FAST-Calib/output:/opt/catkin_ws/src/FAST-Calib/output \
  -v ./ros_overlay/FAST-LIVO2/config:/opt/catkin_ws/src/FAST-LIVO2/config \
  -v ./scripts:/opt/scripts \
  --name fast-livo2 \
  fast-livo2:noetic-r35.4.1
```

Connect to container
```
docker ps -a 
docker exec -it fast-livo2 /bin/bash
```


Launches
```
// start foxglove
roslaunch foxglove_bridge foxglove_bridge.launch

// to test camera
nano /opt/catkin_ws/src/usb_cam/launch/usb_cam-test.launch
roslaunch usb_cam usb_cam-test.launch

// start camera with trigger mode
python3 set_camera_mode.py --trigger 1 --device /dev/video0 --wb 4500 --exposure 2000
roslaunch trigger_cam trigger_cam.launch

// start lidar
roslaunch ouster_ros driver.launch sensor_hostname:=169.254.33.38
roslaunch ouster_ros driver.launch sensor_hostname:=192.168.100.2

// start calib 
// edit config/qr_params.yaml first
roslaunch fast_calib calib.launch
roslaunch fast_calib multi_calib.launch
```



camera mode trigger
https://stackoverflow.com/questions/69055321/image-capturing-on-econ-see3cam-based-on-external-triggers





ptp

sudo nano /etc/systemd/system/ptp4l.service

[Unit]
Description=PTP4L Precision Time Protocol Daemon
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
ExecStart=/usr/sbin/ptp4l -i eth0 -m -4
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target


sudo systemctl daemon-reload
sudo systemctl enable ptp4l.service
sudo systemctl start ptp4l.service
sudo systemctl status ptp4l.service
