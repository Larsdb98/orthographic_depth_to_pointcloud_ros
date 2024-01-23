#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, PointCloud2
import cv_bridge
import cv2
import numpy as np
import sensor_msgs.point_cloud2 as pc2


class OrthDepthToPointcloud():
    def __init__(self):
        rospy.init_node("orthographic_depth_to_pointcloud", anonymous=True)

        # Parameters
        # Parameters for the orthographic camera
        self.ortho_width = rospy.get_param("~ortho_width", 1.0)
        self.depth_image_topic = rospy.get_param("~depth_image_topic", "/vrglasses_for_robots_ros/depth_map")
        self.pointcloud_out_topic = rospy.get_param("~pointcloud_out_topic", "/vrglasses_for_robots_ros/depth_map/points")
        # self.top_coord_world = rospy.get_param('~top_coord_world', 0.0)
        # self.bottom_coord_world = rospy.get_param('~bottom_coord_world', 1.0)
        # self.left_coord_world = rospy.get_param('~left_coord_world', 0.0)
        # self.right_coord_world = rospy.get_param('~right_coord_world', 1.0)

        # Compute edge coords from ortho width
        self.top_coord_world = None
        self.bottom_coord_world = None
        self.left_coord_world = None
        self.right_coord_world = None


        # Subscribe to depth image topic
        self.depth_image_sub = rospy.Subscriber(self.depth_image_topic, Image, self.depth_image_callback)

        # Publisher for PointCloud2
        self.pointcloud_pub = rospy.Publisher(self.pointcloud_out_topic, PointCloud2, queue_size=1)

        # Initialize CvBridge
        self.bridge = cv_bridge.CvBridge()



    def depth_image_callback(self, depth_image_msg):
        try:
            # Convert depth image message to OpenCV image
            depth_image = self.bridge.imgmsg_to_cv2(depth_image_msg, desired_encoding="passthrough")

            # Get image size
            img_height, img_width = depth_image.shape

            # Compute image side coords (corresponds to what is implemented in Vulkan engine)
            height_world_coords = self.ortho_width * (img_height / float(img_width))
            self.top_coord_world = height_world_coords / 2.0
            self.bottom_coord_world = -height_world_coords / 2.0
            self.left_coord_world = -self.ortho_width / 2.0
            self.right_coord_world = self.ortho_width / 2.0


            # Convert world coordinates to pixel coordinates
            top_coord_pixel = int((self.top_coord_world / (self.bottom_coord_world - self.top_coord_world)) * img_height)
            bottom_coord_pixel = int((self.bottom_coord_world / (self.bottom_coord_world - self.top_coord_world)) * img_height)
            left_coord_pixel = int((self.left_coord_world / (self.right_coord_world - self.left_coord_world)) * img_width)
            right_coord_pixel = int((self.right_coord_world / (self.right_coord_world - self.left_coord_world)) * img_width)

            # Crop the depth image based on pixel coordinates
            cropped_depth_image = depth_image[top_coord_pixel:bottom_coord_pixel, left_coord_pixel:right_coord_pixel]

            # Create point cloud from the cropped depth image
            pointcloud = self.create_pointcloud(cropped_depth_image)

            # Publish the point cloud
            self.publish_pointcloud(pointcloud, depth_image_msg.header)

        except Exception as e:
            rospy.logerr("Error processing depth image: {}".format(e))


    def create_pointcloud(self, depth_image):
        # Assuming a simple conversion from depth to point cloud (adjust as needed)
        # Here, we assume that the depth values directly represent the z-coordinates of the points in the point cloud.
        # You may need to calibrate this based on your camera specifications.

        # Create a mask for valid depth values
        valid_depth_mask = depth_image > 0

        # Extract valid depth values and corresponding pixel coordinates
        valid_depth_values = depth_image[valid_depth_mask]
        pixel_coords = np.argwhere(valid_depth_mask)

        print("Pixel coords: {}".format(pixel_coords))

        # Create point cloud array with (x, y, z) coordinates
        pointcloud_data = np.column_stack((pixel_coords[:, 1] + left_coord_pixel, pixel_coords[:, 0] + top_coord_pixel, valid_depth_values))

        # Create PointCloud2 message
        pointcloud_msg = pc2.create_cloud_xyz32(header=self.header, points=pointcloud_data)

        return pointcloud_msg



    def publish_pointcloud(self, pointcloud_msg, header):
        pointcloud_msg.header = header
        self.pointcloud_pub.publish(pointcloud_msg)



    def run(self):
        rospy.spin()






def main():
    try:
        node = OrthDepthToPointcloud()
        node.run()
    except rospy.ROSInterruptException:
        pass 



#########################################
if __name__ == '__main__':
    main()
