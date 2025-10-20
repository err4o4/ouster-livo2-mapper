
docker exec -it a44d61838f13 /bin/bash




source


roslaunch foxglove_bridge foxglove_bridge.launch
roslaunch usb_cam usb_cam-test.launch
roslaunch ouster_ros driver.launch sensor_hostname:=<sensor host name> metadata:=<json file name>
roslaunch ouster_ros driver.launch sensor_hostname:=os-122317001053.local


kill all ros processes 
pkill -f ros



MAC OS

mkdir sources
cd sources

./fast-calib.sh

docker run -it --rm \
  -p 8765:8765 \
  -v ./FAST-Calib:/opt/catkin_ws/src/FAST-Calib \
  fast-livo2:noetic-r35.4.1


Jetson



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
  --name fast-livo2 \
  fast-livo2:noetic-r35.4.1


// Jetson
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
roslaunch trigger_cam trigger_cam.launch

// start lidar
roslaunch ouster_ros driver.launch sensor_hostname:=169.254.33.38

// start calib 
// edit config/qr_params.yaml first
roslaunch fast_calib calib.launch
roslaunch fast_calib multi_calib.launch
```



camera mode trigger
https://stackoverflow.com/questions/69055321/image-capturing-on-econ-see3cam-based-on-external-triggers