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


Lidar standby
curl -i -X PUT http://192.168.100.2/api/v1/sensor/config/operating_mode -H "Content-Type: application/json" --data-raw '"STANDBY"'


PTP

```
sudo systemctl daemon-reload
sudo systemctl enable ptp-sync.service
sudo systemctl start ptp-sync.service
sudo systemctl status ptp-sync.service
sudo systemctl stop ptp-sync.service

sudo journalctl -u ptp-sync.service -f


sudo phc_ctl /dev/ptp0 get
curl http://192.168.100.2/api/v1/time | jq

curl -i -X PUT http://192.168.100.2/api/v1/time/ptp/profile -H "Content-Type: application/json" --data-raw '"gptp"'
sleep 5
curl -i -X PUT http://192.168.100.2/api/v1/time/ptp/profile -H "Content-Type: application/json" --data-raw '"default"'


sudo rm /usr/local/bin/ptp-sync.sh
sudo nano /usr/local/bin/ptp-sync.sh
sudo chmod +x /usr/local/bin/ptp-sync.sh
```


```
#!/usr/bin/env bash
set -e

INTERFACE="eth0"
PHC_DEVICE="/dev/ptp0"
OUSTER_IP="192.168.100.2"

echo "[PTP] Enabling NTP..."
timedatectl set-ntp true || true

echo "[PTP] Waiting for NTP/system clock sync..."

while true; do
    YEAR="$(date +%Y)"
    NTP_SYNCED="$(timedatectl show -p NTPSynchronized --value 2>/dev/null || echo no)"

    if [ "$YEAR" -ge 2024 ] && [ "$NTP_SYNCED" = "yes" ]; then
        echo "[PTP] System time is valid and NTP synchronized."
        date
        break
    fi

    echo "[PTP] Waiting... year=${YEAR}, ntp_synced=${NTP_SYNCED}"
    sleep 1
done

echo "[PTP] Writing system time to RTC devices..."

if [ -e /dev/rtc0 ]; then
    echo "[PTP] Writing to /dev/rtc0..."
    /usr/sbin/hwclock -w -f /dev/rtc0 || echo "[PTP] WARNING: failed to write /dev/rtc0"
fi

if [ -e /dev/rtc1 ]; then
    echo "[PTP] Writing to /dev/rtc1..."
    /usr/sbin/hwclock -w -f /dev/rtc1 || echo "[PTP] WARNING: failed to write /dev/rtc1"
fi

echo "[PTP] RTC status after write:"
/usr/sbin/hwclock -r -f /dev/rtc0 2>/dev/null || true
/usr/sbin/hwclock -r -f /dev/rtc1 2>/dev/null || true

echo "[PTP] Starting ptp4l..."
/usr/sbin/ptp4l -i "$INTERFACE" -m -4 &

sleep 3

echo "[PTP] Starting phc2sys: CLOCK_REALTIME -> ${PHC_DEVICE}..."
/usr/sbin/phc2sys -w -m -s CLOCK_REALTIME -c "$PHC_DEVICE" -O 0 &

sleep 6

echo "[PTP] Setting Ouster profile to gptp..."
/usr/bin/curl -i -X PUT "http://${OUSTER_IP}/api/v1/time/ptp/profile" \
  -H 'Content-Type: application/json' \
  --data-raw '"gptp"'

sleep 5

echo "[PTP] Setting Ouster profile back to default..."
/usr/bin/curl -i -X PUT "http://${OUSTER_IP}/api/v1/time/ptp/profile" \
  -H 'Content-Type: application/json' \
  --data-raw '"default"'

echo "[PTP] Done."
wait
```