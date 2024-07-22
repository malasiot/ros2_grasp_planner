import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import yaml
import cv2
import os
from sam_interfaces.srv import BoxesSeg
from std_msgs.msg import Int32MultiArray as Int32MultiArrayMsg
import numpy as np


class SegmentationClient(Node):
    def __init__(self):
        """Initialize the segmentation client"""
        super().__init__('segmentation_client')
        self.client = self.create_client(BoxesSeg, 'segmentation')
        while not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Service not available, waiting again...')
        self.bridge = CvBridge()
        self.request = BoxesSeg.Request()
        with open('./src/sam/config/params_boxes.yaml', 'rb') as file:
            params = yaml.safe_load(file)

        self.boxes = params['boxes']
        self.min_mask = params['min_mask_region_area']

    def send_image(self, image_path):
        """read the images, send the requests and receive the segmented images as responses"""
        cv_image = cv2.imread(image_path)
        self.request.input_string = os.path.basename(image_path)
        ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        self.request.image = ros_image

        msg_boxes = Int32MultiArrayMsg()
        if self.boxes is not None:
            flat_list = [x for xs in self.boxes for x in xs]
            msg_boxes.data = flat_list
        self.request.boxes = msg_boxes

        future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def callback(self, future):
        """receive and save the responses"""
        try:
            response = future
            self.get_logger().info('Image segmented')
            segmented_image = self.bridge.imgmsg_to_cv2(response.segmented_image, desired_encoding='bgr8')
            filename = f"./segmented_images/{response.frame_id}"
            cv2.imwrite(filename, segmented_image)

        except Exception as e:
            self.get_logger().info(f'Service call failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    segmentation_client = SegmentationClient()
    image_dir = "./dataset"  # Change this to your image directory
    for filename in os.listdir(image_dir):
        image_path = os.path.join(image_dir, filename)
        future = segmentation_client.send_image(image_path)
        segmentation_client.callback(future)

    segmentation_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
