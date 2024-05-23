# TODO:

import open3d as o3d
import numpy as np
import cv2
 
# Function to create depth and color images from a point cloud
def create_images_from_point_cloud(point_cloud, camera_pose, intrinsics):
  # Create a visualizer
  vis = o3d.visualization.Visualizer()
  vis.create_window(visible=False)

  # Add the point cloud to the visualizer
  vis.add_geometry(point_cloud)

  # Set the view control
  ctr = vis.get_view_control()
  param = o3d.camera.PinholeCameraParameters()
  param.extrinsic = camera_pose
  param.intrinsic = intrinsics
  ctr.convert_from_pinhole_camera_parameters(param)

  # Capture depth and color images
  vis.poll_events()
  vis.update_renderer()
  depth_image = vis.capture_depth_float_buffer(True)
  color_image = vis.capture_screen_float_buffer(True)

  # Convert images to numpy arrays
  depth_image = np.asarray(depth_image)
  color_image = np.asarray(color_image)

  vis.destroy_window()
  return depth_image, color_image
 
# Example usage
if __name__ == "__main__":
  pcd = o3d.io.read_point_cloud("/home/jjiao/Desktop/underground_parking/underground_parking_trial_001.pcd")
  print(pcd)
  # o3d.visualization.draw_geometries([pcd], window_name="pointcloud")

  # Define camera pose (4x4 transformation matrix)
  camera_pose = np.array([
      [0.0, 0.0, 1.0,   15.0],
      [-1.0, 0.0, 0.0, -32.0],
      [0.0, -1.0, 0.0, -1.54],
      [0.0, 0.0, 0.0,   1.0]
  ])

  # Define camera intrinsics
  intrinsics = o3d.camera.PinholeCameraIntrinsic()
  intrinsics.set_intrinsics(width=640, height=360, fx=205.46963709898583, fy=205.46963709898583, cx=320.5, cy=180.5)

  # Create depth and color images from the point cloud
  depth_img, color_img = create_images_from_point_cloud(pcd, camera_pose, intrinsics)

  # Save the images
  o3d.io.write_image("/home/jjiao/Desktop/underground_parking/depth_image.png", o3d.geometry.Image(depth_img))
  o3d.io.write_image("/home/jjiao/Desktop/underground_parking/color_image.png", o3d.geometry.Image(color_img))
  print("Images saved.")
