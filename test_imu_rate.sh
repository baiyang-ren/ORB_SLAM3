#!/bin/bash
echo "Testing RealSense D435i IMU data rate..."
timeout 5 rs-sensor-control -c 2>&1 | grep -E "(Gyro|Accel)" | head -20
