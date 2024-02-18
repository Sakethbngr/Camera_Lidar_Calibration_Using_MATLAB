# Camera_Lidar_Calibration_Using_MATLAB

# Step 1: Camera-LIDAR Calibration
1. Calibration Guide: Follow the detailed steps in the [Matlab's Lidar Camera Calibrator Tutorial for camera-LIDAR calibration](https://www.mathworks.com/help/lidar/ug/get-started-lidar-camera-calibrator.html).
2. Camera Intrinsic Parameters: Ensure you have the intrinsic parameters of your camera. Obtain these by following the [Exporting Camera Intrinsic Parameters Tutorial](https://www.mathworks.com/help/vision/ug/using-the-single-camera-calibrator-app.html).
# Step 2: Data Collection Preparation
1. Checkered Board for Calibration: Use a standard checkered board with an even number of squares on one dimension and an odd number on the other. This is for initial camera calibration and Camera-LIDAR calibration.
2. Camera and LIDAR Setup: Connect the camera to your computer. The LIDAR can be connected to the same computer or a robot, accessible via SSH.
3. Running LIDAR Node: Start the LIDAR node to collect point clouds. Do this before running the datacollect.py script.
4.Data Collection with datacollect.py: Execute datacollect.py to capture 20 sets of images and corresponding point clouds, with a 5-second pause between each set. These data sets are used for Camera-LIDAR calibration.

