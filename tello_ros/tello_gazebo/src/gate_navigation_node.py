#!/usr/bin/env python3

import rclpy, time
from rclpy.node import Node
from tello_msgs.msg import GateData
from tello_control_client_node import TelloControlNode


class GateNavigationNode(Node):
    def __init__(self):
        super().__init__('gate_navigation_node')
        self.tello_control_node = TelloControlNode()
        self.subscription = self.create_subscription(GateData, '/drone1/gates', self.listener_callback, 1)
        self.gate_data = None
        self.rotations_counter = 1
        self.movement_counter = 1
        
    def listener_callback(self, msg):
    	self.gate_data = msg

    def navigate_through_gate(self):
        # While loop to keep the drone navigating through the gates
        while True:
            try:
                rclpy.spin_once(self)
                # Calculate the movement commands based on closest gate position
                self.calculate_movement()
            except KeyboardInterrupt:
                self.tello_control_node.Stop();
                rclpy.shutdown()

    def calculate_movement(self):        
        if not (self.gate_data.green_detected or self.gate_data.blue_detected or self.gate_data.red_detected):
            print("No gate detected, rotating...")

            if self.rotations_counter > 11:
                self.rotations_counter = 1
                self.tello_control_node.ChangeAltitude(1)
                print("Ascending")
                return
            
            if self.rotations_counter % 2:
                self.tello_control_node.TurnRight(30 * self.rotations_counter)
            else:
                self.tello_control_node.TurnLeft(30 * self.rotations_counter)
               
            self.rotations_counter = self.rotations_counter + 1
            
            return
            
        self.rotations_counter = 1
        
        gate_data = {
        "green": self.gate_data.green_data if self.gate_data.green_detected else None,
        "red": self.gate_data.red_data if self.gate_data.red_detected else None,
        "blue": self.gate_data.blue_data if self.gate_data.blue_detected else None
        }
    
        # Remove None values
        gate_data = {key: value for key, value in gate_data.items() if value is not None}

        # Calculate distance to each gate and find the closest one
        closest_gate_color = min(gate_data, key=lambda color: gate_data[color][2])

        closest_gate_data = gate_data[closest_gate_color]

        gate_x, gate_y, distance, ratio, height_diff = closest_gate_data
        distance /= 1000
            
        print(f"Gate detected: {gate_x}, {gate_y}")
        # FOV: ~60°, Image width: 960px
        
        # Counter oscillating movement by changing perspective
        if self.movement_counter > 15:
            print("Changing perspective.")
            self.tello_control_node.MoveForward(1)
            self.tello_control_node.TurnLeft(5)
            self.movement_counter = 0
        elif (gate_y < -20):
            print("Ascending...")
            self.tello_control_node.ChangeAltitude(0.1 * -gate_y/20)
        elif (gate_y > 20):
            print("Descending...")
            self.tello_control_node.ChangeAltitude(-0.1 * gate_y/20)
        elif (gate_x > 20):
            print("Moving right...")
            self.tello_control_node.MoveRight(0.075 * gate_x / 20)
        elif (gate_x < -20):
            print("Moving left...")
            self.tello_control_node.MoveLeft(abs(0.075 * gate_x / 20))
        elif ratio < 750:
            if height_diff > 0:
                # Left side larger -> Move right
                print("Gate in incorrect angle. Correct by moving right.")
                self.tello_control_node.MoveRight(0.9)
                self.tello_control_node.TurnLeft(12)
                self.tello_control_node.MoveForward(1.2)
                self.movement_counter = 0
            else:
                print("Gate in incorrect angle. Correct by moving left.")
                self.tello_control_node.MoveLeft(0.9)
                self.tello_control_node.TurnRight(12)
                self.tello_control_node.MoveForward(1.2)
                self.movement_counter = 0
        else:
            if distance > 5:
                distance = distance / 3
            print(f"Moving forward, distance = {distance}")
            self.tello_control_node.MoveForward(distance)
            self.movement_counter = 0
            
        self.movement_counter += 1
         
def main(args=None):
    rclpy.init(args=args)
    gate_navigation_node = GateNavigationNode()
    gate_navigation_node.navigate_through_gate()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
