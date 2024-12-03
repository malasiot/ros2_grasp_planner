import sys

from grasp_planner_msgs.srv import GraspCandidates, GraspCandidatesFilter

import rclpy
from rclpy.node import Node


class ClientAsync(Node):

    def __init__(self):
        super().__init__('client_async')
        self.grasp_candidates_cli = self.create_client(GraspCandidates, 'grasp_candidates_service')
        self.grasp_filter_cli     = self.create_client(GraspCandidatesFilter, 'grasp_candidates_filter_service')

        while not self.grasp_candidates_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Grasp candidates service not available, waiting again...')
        while not self.grasp_filter_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Grasp candidates filter service not available, waiting again...')
                
    def send_request(self):
        self.req = GraspCandidates.Request()
        self.future = self.grasp_candidates_cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        response = self.future.result() ;

        req = GraspCandidatesFilter.Request()
        req.grasps = response.grasps ;
        self.future = self.grasp_filter_cli.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        response = self.future.result() ;

        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_client = ClientAsync()
    response = minimal_client.send_request()
    #minimal_client.get_logger().info(
     #   'Result of add_two_ints: for %d + %d = %d' %
      #  (int(sys.argv[1]), int(sys.argv[2]), response.sum))

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
