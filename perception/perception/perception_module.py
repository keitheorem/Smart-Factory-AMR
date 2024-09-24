import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge               # For converting ROS Image messages to OpenCV images
import cv2  
from ultralytics import YOLO
import numpy as np
from message_filters import Subscriber, ApproximateTimeSynchronizer

class PerceptionModule(Node):

    def __init__(self):
        super().__init__('perception_module')
        self.model = YOLO('yolov8n-seg.pt')  
        self.bridge = CvBridge() 

        # Subscribers to receive data from camera
        self.depth_sub = Subscriber(self, Image, '/depth_cam/depth/image_raw')
        self.camera_info_sub = Subscriber(self, CameraInfo, '/depth_cam/depth/camera_info')
        self.rgb_sub = Subscriber(self, Image, '/depth_cam/rgb/image_raw')

        # Synchronize depth and camera info messages
        self.sync = ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub, self.camera_info_sub], queue_size=30, slop=0.1)
        self.sync.registerCallback(self.sync_callback)
        
        # Publish data after segmentation through yolov8
        self.mask_publisher = self.create_publisher(Image, '/perception/combined_mask', 10)
        self.depth_publisher = self.create_publisher(Image, 'filtered_depth', 10)
        self.camera_info_publisher = self.create_publisher(CameraInfo, '/camera_info', 10)

        self.pixel_positions = None
        self.latest_depth_image = None

    def sync_callback(self, rgb_msg, depth_msg, camera_info_msg):
        try:
            # Convert ROS2 Image message to cv2 image
            image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')

            # Run YOLOv8 segmentation on the captured frame
            results = self.model(source=image, conf=0.2, device='cuda')

            # Get masks data results
            masks = results[0].masks.data

            if masks is not None and len(masks) > 0:
                pixel_positions_list = []
                for mask in masks:
                    # Convert mask from tensor to numpy array and squeeze dimension
                    mask = mask.squeeze().cpu().numpy()

                    # Convert the mask to 8-bit grayscale and rescale (0 or 255)
                    mask_img = (mask * 255).astype('uint8')

                    # Get the pixel positions where the mask has a value of 1
                    pixel_positions = np.argwhere(mask_img == 255)
                    pixel_positions_list.append(pixel_positions)

                # Store the pixel positions to be used in depth callback if there are valid masks
                self.pixel_positions = np.concatenate(pixel_positions_list)

                # Convert the single-channel mask to a 3-channel BGR image
                mask_img = cv2.cvtColor(mask_img, cv2.COLOR_GRAY2BGR)

                # Convert the combined mask (OpenCV image) to ROS2 Image message for publishing
                mask_msg = self.bridge.cv2_to_imgmsg(mask_img, encoding="bgr8")

                # Publish the combined mask
                self.mask_publisher.publish(mask_msg)
            else:
                # If no masks are detected, set pixel_positions to None
                self.pixel_positions = None

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

        try:
            # Check if pixel_positions is valid before filtering depth image
            if self.pixel_positions is not None and len(self.pixel_positions) > 0:
                # Convert ROS2 Depth Image message to OpenCV image (depth in meters)
                depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

                # Create a mask to filter out unwanted pixels (initialize as zeros)
                filtered_depth = np.zeros_like(depth_image)

                # Iterate through the pixel positions from the masks
                for (y, x) in self.pixel_positions:
                    if 0 <= y < depth_image.shape[0] and 0 <= x < depth_image.shape[1]:
                        # Copy the depth value at the mask's pixel position to the filtered depth image
                        filtered_depth[y, x] = depth_image[y, x]

                # Convert the filtered depth image back to a ROS2 Image message
                filtered_depth_msg = self.bridge.cv2_to_imgmsg(filtered_depth, encoding="passthrough")

                # Publish the filtered depth image and camera info
                self.depth_publisher.publish(filtered_depth_msg)
                self.camera_info_publisher.publish(camera_info_msg)
            else:
                self.get_logger().info("No valid pixel positions to filter depth image.")

        except Exception as e:
            self.get_logger().error(f"Error processing synchronized depth image: {e}")


def main(args=None):
    rclpy.init(args=args)
    perception_module = PerceptionModule()
    rclpy.spin(perception_module)
    perception_module.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()