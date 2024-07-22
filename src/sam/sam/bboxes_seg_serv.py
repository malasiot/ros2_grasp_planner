import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml
from sam_interfaces.srv import BoxesSeg
from sam import SAMbboxes, combine_masks_boxes

# Import SAM related modules


class SegmentationService(Node):
    def __init__(self):
        """
        load the config file and initialize sam
        """
        super().__init__('segmentation_service')
        self.srv = self.create_service(BoxesSeg, 'segmentation', self.segmentation_callback)
        self.bridge = CvBridge()

        with open('./src/sam/config/params_boxes.yaml', 'rb') as file:
            params = yaml.safe_load(file)

        self.sam = SAM_bboxes('vit_h', 'cuda:0')
        self.min_mask = params['min_mask_region_area']

    def segmentation_callback(self, request, response):
        """ Obtain the file name from the request """

        cv_image = self.bridge.imgmsg_to_cv2(request.image, desired_encoding='bgr8')
        # Read the image from the file
        boxes = np.asarray(request.boxes.data).reshape((len(request.boxes.data) // 4, 4)) if len(request.boxes.data) > 0 else None
        # Perform segmentation with SAM model
        print(boxes)
        masks, scores, _ = self.sam.segment(cv_image, boxes)

        # Combine masks into a single segmented image
        segmented_image = combine_masks_boxes(masks, scores, cv_image, request.input_string, self.min_mask)

        blended_image = cv2.addWeighted(cv_image, 0.4, segmented_image, 0.6, 0)
        # Convert segmented image to ROS Image message
        segmented_msg = self.bridge.cv2_to_imgmsg(blended_image, encoding='bgr8')
        response.segmented_image = segmented_msg
        response.frame_id = request.input_string

        return response


def main(args=None):
    rclpy.init(args=args)
    segmentation_service = SegmentationService()
    rclpy.spin(segmentation_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
