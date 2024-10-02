import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import sys
import argparse
from grasp_planner_interfaces.srv import Segmentation


class SegmentationClient(Node):
    def __init__(self):
        """Initialize the segmentation client"""
        super().__init__('segmentation_client')
        self.client = self.create_client(Segmentation, 'segmentation')
        while not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Service not available, waiting again...')
        self.bridge = CvBridge()
        self.request = Segmentation.Request()

    def send_image(self, image_path):
        """read the images, send the requests and receive the segmented images as responses"""
        cv_image = cv2.imread(image_path)
        self.request.input_string = os.path.basename(image_path)
        ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        self.request.image = ros_image
        future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def callback(self, future):
        """receive and save the responses"""
        try:
            response = future
            self.get_logger().info('Image segmented')
            segmented_image = self.bridge.imgmsg_to_cv2(response.segmented_image, desired_encoding='bgr8')
            # scores = response.scores

            filename = f"./segmented_images/result.png"
            cv2.imwrite(filename, segmented_image)
        except Exception as e:
            self.get_logger().info(f'Service call failed: {e}')


def main(args=None):

    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument('filenames', metavar='f', type=str, nargs='+',
                    help='files to process')
   
    rclpy.init(args=args)

    args = parser.parse_args() ;

    segmentation_client = SegmentationClient()
  

    for image_path in args.filenames:
        future = segmentation_client.send_image(image_path)
        segmentation_client.callback(future)

    segmentation_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
