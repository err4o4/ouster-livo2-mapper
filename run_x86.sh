export DISPLAY=:0
xhost +local:root
docker run -it --rm \
  --network host \
  --gpus all \
  -p 8765:8765 \
  -e DISPLAY=${DISPLAY:-:0} \
  -e XAUTHORITY=/root/.Xauthority \
  -e QT_X11_NO_MITSHM=1 \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $HOME/.Xauthority:/root/.Xauthority:ro \
  -v ./ros_overlay/usb_cam/launch:/opt/catkin_ws/src/usb_cam/launch \
  -v ./ros_overlay/FAST-Calib/config:/opt/catkin_ws/src/FAST-Calib/config \
  -v ./ros_overlay/FAST-Calib/output:/opt/catkin_ws/src/FAST-Calib/output \
  -v ./ros_overlay/FAST-Calib/calib_data:/opt/catkin_ws/src/FAST-Calib/calib_data \
  -v ./ros_overlay/LiDAR_IMU_Init/config:/opt/catkin_ws/src/LiDAR_IMU_Init/config \
  -v ./ros_overlay/LiDAR_IMU_Init/launch:/opt/catkin_ws/src/LiDAR_IMU_Init/launch \
  -v ./ros_overlay/LiDAR_IMU_Init/result:/opt/catkin_ws/src/LiDAR_IMU_Init/result \
  -v ./ros_overlay/FAST_LIO/config:/opt/catkin_ws/src/FAST_LIO/config \
  -v ./ros_overlay/FAST-LIVO2/config:/opt/catkin_ws/src/FAST-LIVO2/config \
  -v ./scripts:/opt/scripts \
  --name hku_mars_x86 \
  hku_mars_x86