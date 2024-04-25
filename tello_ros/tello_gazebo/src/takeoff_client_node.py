#!/usr/bin/env python3

import rclpy
from tello_msgs.srv import TelloAction  # Import the service message
from rclpy.node import Node

class TakeoffClient(Node):
    def __init__(self):
        super().__init__('takeoff_client')

        # Create service client
        self.client = self.create_client(TelloAction, '/drone1/tello_action')

        # Wait until the service is available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

    def send_takeoff_request(self):
        # Prepare request message
        request = TelloAction.Request()
        request.cmd = 'takeoff'

        # Send the service call
        future = self.client.call_async(request)

        # Wait for the result
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info('Takeoff command sent successfully')
        else:
            self.get_logger().error('Failed to send takeoff command')

def main(args=None):
    rclpy.init(args=args)
    takeoff_client = TakeoffClient()
    takeoff_client.send_takeoff_request()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
