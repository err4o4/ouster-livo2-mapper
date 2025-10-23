docker run -it --rm \
  -p 8765:8765 \
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
  --name hku_mars_arm64 \
  hku_mars_arm64