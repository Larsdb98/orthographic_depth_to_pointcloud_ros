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

            inv_aspect_ratio = img_height/float(img_width)

            # Compute image side coords (corresponds to what is implemented in Vulkan engine)
            height_world_coords = self.ortho_width * (img_height / float(img_width))
            self.top_coord_world = height_world_coords / 2.0
            self.bottom_coord_world = -height_world_coords / 2.0
            self.left_coord_world = -self.ortho_width / 2.0
            self.right_coord_world = self.ortho_width / 2.0


            remapped_coords = np.meshgrid(
                np.linspace(self.left_coord_world, self.right_coord_world, img_width),
                # np.linspace(self.top_coord_world, self.bottom_coord_world, img_height)
                np.linspace(self.bottom_coord_world, self.top_coord_world, img_height)
            )

            # Filter out invalid depth values
            valid_depth_mask = depth_image > 0.0
            remapped_coords = [coord[valid_depth_mask] for coord in remapped_coords]
            depth_values = depth_image[valid_depth_mask]

            pointcloud_data = np.column_stack((
                remapped_coords[0],
                remapped_coords[1],
                depth_values
            ))

            print("Depth values: {}".format(depth_values))

            # Create point cloud from the cropped depth image
            pointcloud_msg = pc2.create_cloud_xyz32(header=depth_image_msg.header, points=pointcloud_data)

            # Publish the point cloud
            self.publish_pointcloud(pointcloud_msg, depth_image_msg.header)

        except Exception as e:
            rospy.logerr("Error processing depth image: {}".format(e))




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
