import os
import rospy
from message_filters import Subscriber, ApproximateTimeSynchronizer
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np
from scipy.spatial.transform import Rotation

class RGBD_Collector():

	def __init__(self):
		self.cnt = 0

	def create_directories(self):
		self.rgb_path = '/Titan/dataset/cloudrender/test_underground_parking/data_to_hd/rgb_frame'
		os.makedirs(self.rgb_path, exist_ok=True)
		self.depth_path = '/Titan/dataset/cloudrender/test_underground_parking/data_to_hd/depth_frame'
		os.makedirs(self.depth_path, exist_ok=True)
		self.pose_path = '/Titan/dataset/cloudrender/test_underground_parking/data_to_hd/pose.txt'
		
		self.last_timestamp = 0.0
		self.save_fps = 1.0

	def callback(self, rgb_msg, depth_msg, odom_msg):
			if abs(rgb_msg.header.stamp.to_sec() - self.last_timestamp) > 1.0 / self.save_fps:
				self.last_timestamp = rgb_msg.header.stamp.to_sec()

				bridge = CvBridge()

				# Convert ROS Image messages to OpenCV images
				rgb_image = bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
				depth_image = bridge.imgmsg_to_cv2(depth_msg, '32FC1')

				# Save the images
				timestamp = rospy.Time.now().to_nsec()
				rgb_image_path = '{}/{:06}.png'.format(self.rgb_path, self.cnt)
				depth_image_path = '{}/{:06}.png'.format(self.depth_path, self.cnt)
				
				cv2.imwrite(rgb_image_path, rgb_image)
				cv2.imwrite(depth_image_path, depth_image)

				# Save the pose
				with open(self.pose_path, 'a') as pose_file:
						pose = odom_msg.pose.pose
						
						quat_w2s = np.array([pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z])
						R_w2s = Rotation.from_quat(np.roll(quat_w2s, -1)).as_matrix()
						trans_w2s = np.array([pose.position.x, pose.position.y, pose.position.z])

						camera_offset_z = 0.5
						if (rospy.get_param('/vehicleSimulator/cameraOffsetZ')):
							camera_offset_z = rospy.get_param('/vehicleSimulator/cameraOffsetZ')
						R_s2c = Rotation.from_quat(np.roll([0.5, -0.5, 0.5, -0.5], -1)).as_matrix()
						trans_s2c = np.array([0.0, 0.0, camera_offset_z])

						R_w2c = R_w2s @ R_s2c
						quat_w2c = np.roll(Rotation.from_matrix(R_w2c).as_quat(), 1)				
						trans_w2c = R_w2s @ trans_s2c + trans_w2s

						# print(quat_w2c)
						# print(trans_w2c)
						pose_file.write('{:3f} {:3f} {:3f} {:3f} {:3f} {:3f} {:3f}\n'.format(\
							quat_w2c[0], quat_w2c[1], quat_w2c[2], quat_w2c[3], trans_w2c[0], trans_w2c[1], trans_w2c[2]))
				self.cnt += 1
				print('Save {}th RGBD images and poses to {} at time {:3f}'.format(self.cnt, self.rgb_path, rgb_msg.header.stamp.to_sec()))

	def initialize_ros_node(self):
			rospy.init_node('data_synchronizer', anonymous=True)

			# Subscribers
			rgb_sub = Subscriber('/cloudrender_camera/color/image', Image)
			depth_sub = Subscriber('/cloudrender_camera/depth/image', Image)
			pose_sub = Subscriber('/state_estimation', Odometry)

			# ApproximateTimeSynchronizer
			ats = ApproximateTimeSynchronizer([rgb_sub, depth_sub, pose_sub], queue_size=10, slop=0.1)
			ats.registerCallback(self.callback)

def main():
		rgbd_collector = RGBD_Collector()
		rgbd_collector.create_directories()
		rgbd_collector.initialize_ros_node()
		rospy.spin()

if __name__ == '__main__':
		main()
