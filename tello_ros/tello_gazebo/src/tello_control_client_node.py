#!/usr/bin/env python3

import rclpy, time
from tello_msgs.srv import TelloAction  # Import the service message
from rclpy.node import Node

class TelloControlNode(Node):
    def __init__(self):
        super().__init__('tello_movement_control_node')

        # Create service client
        self.client = self.create_client(TelloAction, '/drone1/tello_action')

        # Wait until the service is available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
    
    def Stop(self):
        request = TelloAction.Request()
        request.cmd = 'rc 0 0 0 0'
        self.client.call_async(request)

    def TurnLeft(self, angle_in_degrees = 90):
        request = TelloAction.Request()
        request.cmd = 'rc 0 0 0 0.3'
        m_time = 10 * angle_in_degrees/360
        self.client.call_async(request)
        time.sleep(m_time)
        request.cmd = 'rc 0 0 0 0'
        self.client.call_async(request)
        time.sleep(1.2)

    def TurnRight(self, angle_in_degrees = 90):
        request = TelloAction.Request()
        request.cmd = 'rc 0 0 0 -0.3'
        m_time = 10 * angle_in_degrees/360
        self.client.call_async(request)
        time.sleep(m_time)
        request.cmd = 'rc 0 0 0 0'
        self.client.call_async(request)
        time.sleep(1.2)

    def TurnLeftSlowly(self):
        request = TelloAction.Request()
        request.cmd = 'rc 0 0 0 0.03'
        m_time = 0.5
        self.client.call_async(request)
        time.sleep(m_time)
        request.cmd = 'rc 0 0 0 0'
        self.client.call_async(request)
        time.sleep(0.5)
    
    def TurnRightSlowly(self):
        request = TelloAction.Request()
        request.cmd = 'rc 0 0 0 -0.03'
        m_time = 0.5
        self.client.call_async(request)
        time.sleep(m_time)
        request.cmd = 'rc 0 0 0 0'
        self.client.call_async(request)
        time.sleep(0.5)

    def MoveForward(self, distance = 1):
        request = TelloAction.Request()
        request.cmd = 'rc 0.4 0 0 0'
        m_time = distance / 4
        self.client.call_async(request)
        time.sleep(m_time)
        request.cmd = 'rc 0 0 0 0'
        self.client.call_async(request)
        time.sleep(0.8)

    def MoveBackward(self, distance = 1):
        request = TelloAction.Request()
        request.cmd = 'rc -0.4 0 0 0'
        m_time = distance / 4
        self.client.call_async(request)
        time.sleep(m_time)
        request.cmd = 'rc 0 0 0 0'
        self.client.call_async(request)
        time.sleep(0.8)

    def MoveLeft(self, distance = 1):
        request = TelloAction.Request()
        request.cmd = 'rc 0 0.4 0 0'
        m_time = distance / 4
        self.client.call_async(request)
        time.sleep(m_time)
        request.cmd = 'rc 0 0 0 0'
        self.client.call_async(request)
        time.sleep(1)

    def MoveRight(self, distance = 1):
        request = TelloAction.Request()
        request.cmd = 'rc 0 -0.4 0 0'
        m_time = distance / 4
        self.client.call_async(request)
        time.sleep(m_time)
        request.cmd = 'rc 0 0 0 0'
        self.client.call_async(request)
        time.sleep(1)

    def ChangeAltitude(self, change_value = 0):
        if change_value != 0:
            request = TelloAction.Request()
            if change_value < 0:
                request.cmd = 'rc 0 0 -0.4 0'
            else:
                request.cmd = 'rc 0 0 0.4 0'
            m_time = abs(change_value / 4)
            self.client.call_async(request)
            time.sleep(m_time)
            request.cmd = 'rc 0 0 0 0'
            self.client.call_async(request)
            time.sleep(0.8)

