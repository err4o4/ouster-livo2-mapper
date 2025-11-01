#!/bin/bash
# Diagnostic script for FAST_LIO point cloud issues

echo "=== FAST_LIO Point Cloud Diagnostic ==="
echo ""
echo "1. Checking raw Ouster point cloud..."
rostopic hz /velodyne_points_HDL32 &
HZ_PID=$!
sleep 5
kill $HZ_PID 2>/dev/null

echo ""
echo "2. Counting points in one scan..."
rostopic echo /velodyne_points_HDL32/width -n 1

echo ""
echo "3. Checking point cloud fields..."
rostopic echo /velodyne_points_HDL32/fields -n 1