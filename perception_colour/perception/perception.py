#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node 
import numpy as np
from message_filters import Subscriber, ApproximateTimeSynchronizer
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge               # For converting ROS Image messages to OpenCV images
import cv2 

class Perception(Node):

    def __init__(self):
        super().__init__('perception_module')

        # Subscribe to camera data 
        self.bridge = CvBridge() 
        self.depth_sub = Subscriber(self, Image, '/depth_cam/depth/image_raw')
        self.camera_info_sub = Subscriber(self, CameraInfo, '/depth_cam/depth/camera_info')
        self.rgb_sub = Subscriber(self, Image, '/depth_cam/rgb/image_raw')

        # Synchronize depth and camera info messages
        self.sync = ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub, self.camera_info_sub], queue_size=30, slop=0.1)
        self.sync.registerCallback(self.sync_callback)

        # Publish RGB image
        self.image_publisher = self.create_publisher(Image, '/perception/processed_image', 10)
        self.depth_publisher = self.create_publisher(Image, '/perception/processed_depth', 10)
        self.camera_info_publisher = self.create_publisher(CameraInfo, '/perception/camera_info', 10)
    
    def sync_callback(self, rgb_msg, depth_msg, camera_info_msg):
        # Convert ROS2 Image message to cv2 image
        image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

        # Filter and highlight the largest red object with an oriented bounding box, its center, and angle
        processed_image, processed_depth, depth_center = self.filter_largest_red_object_with_center_and_angle(image, depth_image)

        # Convert cv2 image back to ROS2 Image message
        processed_image_msg = self.bridge.cv2_to_imgmsg(processed_image, encoding="bgr8")
        processed_depth_msg = self.bridge.cv2_to_imgmsg(processed_depth, encoding="passthrough")

        # Publish processed data 
        self.image_publisher.publish(processed_image_msg)
        self.depth_publisher.publish(processed_depth_msg)
        self.camera_info_publisher.publish(camera_info_msg)

    def filter_largest_red_object_with_center_and_angle(self, frame, depth_image):
        # Convert the frame from BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Define the range of red color in HSV
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        lower_blue = np.array([100, 150, 100])
        upper_blue = np.array([140, 255, 255])

        # Create masks for red color
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask3 = cv2.inRange(hsv, lower_blue, upper_blue)
        
        # Combine both masks
        mask = mask1 + mask2 + mask3
        
        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Find the largest contour by area
            largest_contour = max(contours, key=cv2.contourArea)
            
            # Filter by area to avoid small noise
            if cv2.contourArea(largest_contour) > 100:
                # Get the minimum area rectangle (oriented bounding box)
                rect = cv2.minAreaRect(largest_contour) 
                
                # Get box points (coordinates of the four corners of the oriented bounding box)
                box = cv2.boxPoints(rect)
                box = np.int0(box)  # Convert to integer
                
                # Draw the oriented bounding box
                cv2.drawContours(frame, [box], 0, (0, 255, 0), 2)
                
                # Calculate the area of the bounding box
                width, height = rect[1]
                area = width * height
                print(f"Bounding Rectangle Area: {area} pixels")
                
                if area > 4000: 
                    print("Cube Detected")
                    
                # Get the center of the bounding box
                cx, cy = np.int0(rect[0])  # Rect center is the center of the bounding box
                
                # Draw a circle at the center of the largest red object
                cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)  # Blue circle
                
                # Get the angle of rotation of the bounding box
                angle = rect[2]
                if width < height:
                    angle = angle + 90  # Adjust angle if the height is greater than width
                
                # Display the angle
                print(f"Rotation Angle: {angle} degrees")
                
                # Display the angle on the frame
                cv2.putText(frame, f"Angle: {angle:.2f} deg", (cx - 50, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

                # Filter the depth image using the mask (mask should be applied to depth)
                masked_depth = np.where(mask == 0, 0, depth_image)
            
                 # Display some depth values (Optional, for debugging purposes)
                depth_value_at_center = depth_image[cy, cx]

        return frame, masked_depth, depth_value_at_center

def main(args=None):
    rclpy.init(args=args)
    perception_module = Perception()
    rclpy.spin(perception_module)
    perception_module.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()