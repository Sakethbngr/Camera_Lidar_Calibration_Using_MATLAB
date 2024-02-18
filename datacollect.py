import pyrealsense2 as rs
import open3d as o3d
import numpy as np
import cv2
import os
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from scipy.io import savemat

# Create a RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
# Configure the pipeline to stream at the given resolution and format for color and depth streams
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Start streaming with the given configuration
profile = pipeline.start(config)

# Get the color stream profile and extract the intrinsic parameters
color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
color_intrinsics = color_profile.get_intrinsics()

# Create the intrinsic matrix in the MATLAB format
intrinsic_matrix = np.array([[color_intrinsics.fx, 0, color_intrinsics.ppx],
                             [0, color_intrinsics.fy, color_intrinsics.ppy],
                             [0, 0, 1]])

lidar_data = None

def lidar_callback(msg):
    global lidar_data
    lidar_data = msg

# Initialize the ROS node
rospy.init_node('lidar_capture_node')
# Subscribe to the LiDAR data topic
lidar_sub = rospy.Subscriber("/velodyne_points", PointCloud2, lidar_callback)

# Create a directory to store captured data
output_dir = "captured_data"
os.makedirs(output_dir, exist_ok=True)

# Define the rate at which to process the loop
rate = rospy.Rate(10)  # 10 Hz

# Capture 20 instances of images and LiDAR data
for i in range(20):
    rospy.sleep(5)

    # Capture a frame from the RealSense camera
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()
    
    # Check for valid frames
    if not color_frame or not depth_frame:
        continue

    color_image = np.asanyarray(color_frame.get_data())

    # Wait until LiDAR data is received
    while lidar_data is None and not rospy.is_shutdown():
        print("Waiting for LIDAR data...")
        rospy.sleep(1)
        rate.sleep()

    # Convert LiDAR data to a point cloud if data is available
    if lidar_data is not None:
        pcl_data = list(point_cloud2.read_points(lidar_data, field_names=("x", "y", "z"), skip_nans=True))
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(pcl_data)

        # Save LiDAR data in PCD format
        pcd_filename = os.path.join(output_dir, f"lidar_{i}.pcd")
        o3d.io.write_point_cloud(pcd_filename, cloud)
        print(f"Saved {pcd_filename}")

    # Save the captured color image
    image_filename = os.path.join(output_dir, f"image_{i}.png")
    cv2.imwrite(image_filename, color_image)
    print(f"Saved {image_filename}")

    # Reset LiDAR data
    lidar_data = None

# Save the intrinsic matrix to a .mat file
savemat(os.path.join(output_dir, 'color_intrinsics.mat'), {'intrinsicMatrix': intrinsic_matrix})
print(f"Saved color_intrinsics.mat in {output_dir}")

# Stop the RealSense pipeline
pipeline.stop()
