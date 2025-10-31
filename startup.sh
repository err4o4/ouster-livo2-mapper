#!/bin/bash
set -e

# Source ROS environment
source /opt/catkin_ws/devel/setup.bash

echo "Starting ROS launch files..."

# Function to handle cleanup on exit
cleanup() {
    echo "Shutting down ROS nodes..."
    killall -9 rosmaster roscore roslaunch 2>/dev/null || true
    exit 0
}

trap cleanup SIGTERM SIGINT

# Start roscore if not running
if ! pgrep -x roscore > /dev/null; then
    echo "Starting roscore..."
    roscore &
    sleep 3
fi

# Launch foxglove_bridge in background
echo "Launching foxglove_bridge..."
roslaunch foxglove_bridge foxglove_bridge.launch > /tmp/foxglove.log 2>&1 &
FOXGLOVE_PID=$!

# Give it a moment to start
sleep 2

# Launch ros_supervisor in background
echo "Launching ros_supervisor..."
roslaunch ros_supervisor supervisor.launch > /tmp/supervisor.log 2>&1 &
SUPERVISOR_PID=$!

echo ""
echo "======================================"
echo "ROS nodes started successfully!"
echo "  - foxglove_bridge (PID: $FOXGLOVE_PID)"
echo "  - ros_supervisor (PID: $SUPERVISOR_PID)"
echo "======================================"
echo "Logs available at:"
echo "  - /tmp/foxglove.log"
echo "  - /tmp/supervisor.log"
echo "======================================"
echo ""

# Check if running in interactive mode (has a TTY)
if [ -t 0 ]; then
    echo "Interactive mode detected. Starting bash shell..."
    exec /bin/bash
else
    echo "Detached mode detected. Keeping container alive..."
    # Keep container running by waiting on background processes
    wait $FOXGLOVE_PID $SUPERVISOR_PID
fi
