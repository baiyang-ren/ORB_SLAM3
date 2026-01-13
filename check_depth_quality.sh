#!/bin/bash
echo "Checking RealSense D435i depth quality..."
echo ""
echo "Run 'realsense-viewer' and check:"
echo "1. Enable 'Depth' stream - should show grayscale depth image"
echo "2. Enable 'Color' stream - should show RGB image"
echo "3. Look for:"
echo "   - Black holes/gaps in depth (bad)"
echo "   - Flickering depth values (bad)"
echo "   - Smooth, consistent depth (good)"
echo "4. Point camera at textured surface 1-2m away"
echo ""
echo "Starting realsense-viewer..."
realsense-viewer
