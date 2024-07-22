import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml
from pycocotools import mask
from grasp_planner_interfaces.srv import Segmentation
import pickle
from sam import SAMpoints
from sam import combine_masks_points
import torch
from threading import Timer
from ament_index_python.packages import get_package_share_directory

# Import SAM related modules
from segment_anything import SamAutomaticMaskGenerator, sam_model_registry


class SegmentationService(Node):
    def __init__(self):
        """
        load the config file and initialize sam
        """
        super().__init__('segmentation_service')
        self.srv = self.create_service(Segmentation, 'segmentation', self.segmentation_callback)
        self.bridge = CvBridge()

        self.declare_parameters(
            namespace='',
            parameters=[
                ('model_type', 'None'),
                ('checkpoint', './model/sam_vit_h_4b8939.pth'),
                ('points_per_side', 50),
                ('points_per_batch', 'None'),
                ('pred_iou_thresh', 0.92),
                ('stability_score_thresh', 0.9),
                ('stability_score_offset', 'None'),
                ('box_nms_thresh', 0.2),
                ('crop_n_layers', 'None'),
                ('crop_nms_thresh', 0.8),
                ('crop_overlap_ratio', 'None'),
                ('crop_n_points_downscale_factor', 'None'),
                ('min_mask_region_area', 8000),
                ('device', 'cuda'),
                ('output_mode', 'coco_rle')
            ]
        )

        param_dict = self.get_args()

        self.sam = SAMpoints(param_dict, self.get_parameter("model_type").value, self.get_parameter("device").value)
        self.min_mask = param_dict['min_mask_region_area']

        self.get_logger().info(f'Model loaded')

        self.last_request_time = None
   #     self.shutdown_timer = Timer(30.0, self.shutdown_if_idle)  # Adjust the timeout as needed
   #     self.shutdown_timer.start()

    def segmentation_callback(self, request, response):
        """ Obtain the file name from the request """

        self.last_request_time = rclpy.clock.Clock().now()
        cv_image = self.bridge.imgmsg_to_cv2(request.image, desired_encoding='bgr8')

        # Perform segmentation with SAM model
        result = self.sam.segment(cv_image)

        # Combine masks into a single segmented image
        segmented_image = combine_masks_points(result, cv_image, request.input_string, self.min_mask)

        #blended_image = cv2.addWeighted(cv_image, 0.4, segmented_image, 0.6, 0)
        # Convert segmented image to ROS Image message
        segmented_msg = self.bridge.cv2_to_imgmsg(segmented_image, encoding='bgr8')
        response.segmented_image = segmented_msg
        response.frame_id = request.input_string

        return response

    def shutdown_if_idle(self):
        """
        Shutdown the node if no requests have been received recently
        """
        current_time = rclpy.clock.Clock().now()
        time_since_last_request = (current_time - self.last_request_time).nanoseconds
        if time_since_last_request*10**9 > 30.0:  # 30 seconds
            self.get_logger().info("No requests received for 30 seconds. Shutting down...")
            rclpy.shutdown()

    def get_args(self):
        # Define a list of parameter names
        param_names = [
            "points_per_side",
            "points_per_batch",
            "pred_iou_thresh",
            "stability_score_thresh",
            "stability_score_offset",
            "box_nms_thresh",
            "crop_n_layers",
            "crop_nms_thresh",
            "crop_overlap_ratio",
            "crop_n_points_downscale_factor",
            "min_mask_region_area"
        ]

        args = {}
        loaded_params = self.get_parameters(param_names)
        for param in loaded_params:
            param_name = param.name
            param_value = param.value
            if param.value != 'None':
                args[param_name] = param_value

        # Print the loaded parameters
        for k, v in args.items():
            print(f"{k}: {v}")

        return args


def main(args=None):
    rclpy.init(args=args)
    segmentation_service = SegmentationService()
    rclpy.spin(segmentation_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
