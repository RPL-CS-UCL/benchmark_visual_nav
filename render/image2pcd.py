#!/usr/bin/env python

import rospy
import numpy as np
import open3d as o3d
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2

class DepthImageToPointCloud:
    def __init__(self):
        self.bridge = CvBridge()
        self.intrinsic_matrix = None
        self.depth_image = None
        
        rospy.init_node('depth_image_to_pointcloud', anonymous=True)

        self.image_sub = rospy.Subscriber('/cloudrender_camera/depth/image', Image, self.image_callback)
        self.info_sub = rospy.Subscriber('/cloudrender_camera/camera_info', CameraInfo, self.info_callback)

    def image_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        if self.intrinsic_matrix is not None:
            self.generate_pointcloud()

    def info_callback(self, msg):
        K = msg.K
        self.intrinsic_matrix = np.array(K).reshape(3, 3)

    def generate_pointcloud(self):
        if self.depth_image is None or self.intrinsic_matrix is None:
            return

        height, width = self.depth_image.shape
        fx = self.intrinsic_matrix[0, 0]
        fy = self.intrinsic_matrix[1, 1]
        cx = self.intrinsic_matrix[0, 2]
        cy = self.intrinsic_matrix[1, 2]
        print(height, width)
        print(self.intrinsic_matrix)

        points = []
        for v in range(height):
            for u in range(width):
                z = self.depth_image[v, u]
                if z == 0:
                    continue
                x = (u - cx) * z / fx
                y = (v - cy) * z / fy
                points.append((x, y, z))

        # Create Open3D point cloud
        pointcloud = o3d.geometry.PointCloud()
        pointcloud.points = o3d.utility.Vector3dVector(points)

        # Save the point cloud to a file
        o3d.io.write_point_cloud("/Titan/dataset/cloudrender/test_underground_parking/depth_points.pcd", pointcloud)
        rospy.loginfo("Point cloud saved as output_pointcloud.pcd")

if __name__ == '__main__':
    try:
        DepthImageToPointCloud()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
