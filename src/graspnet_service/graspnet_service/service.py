
# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from grasp_planner_interfaces.srv import GraspNet
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node

class GraspNetService(Node):

    def __init__(self):
        super().__init__('graspnet_service_node')
        self.srv = self.create_service(GraspNet, 'graspnet', self.graspnet_callback)

    def graspnet_callback(self, request, response):
        bridge = CvBridge()
        rgb_image = bridge.imgmsg_to_cv2(request.rgb, desired_encoding='bgr8')
        depth_image = bridge.imgmsg_to_cv2(request.depth, desired_encoding='mono16')
        camera_info = request.camera_info

        print(rgb_image) ;
      
        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = GraspNetService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
