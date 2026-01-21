#!/usr/bin/env python3
# filepath: test_homing_server.py

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import time

class HomingServiceServer(Node):
    def __init__(self):
        super().__init__('homing_service_server')
        
        # Sensing robot service
        self.sensing_srv = self.create_service(
            SetBool,
            '/xbotcore/homing/switch1',
            self.sensing_homing_callback
        )
        
        # Cleaning robot service
        self.cleaning_srv = self.create_service(
            SetBool,
            '/xbotcore/homing/switch2',
            self.cleaning_homing_callback
        )
        
        self.get_logger().info('Homing service servers started!')
        self.get_logger().info('  - /xbotcore/homing/switch1')
        self.get_logger().info('  - /xbotcore/homing/switch2')

    def sensing_homing_callback(self, request, response):
        self.get_logger().info(f'SENSING homing request: {request.data}')
        
        # Simulate homing delay
        time.sleep(0.5)
        
        if request.data:
            response.success = True
            response.message = "Sensing robot homing completed"
            self.get_logger().info('SENSING homing SUCCESS')
        else:
            response.success = False
            response.message = "Sensing robot homing cancelled"
            self.get_logger().warn('SENSING homing CANCELLED')
        
        return response

    def cleaning_homing_callback(self, request, response):
        self.get_logger().info(f'CLEANING homing request: {request.data}')
        
        # Simulate homing delay
        time.sleep(0.5)
        
        if request.data:
            response.success = True
            response.message = "Cleaning robot homing completed"
            self.get_logger().info('CLEANING homing SUCCESS')
        else:
            response.success = False
            response.message = "Cleaning robot homing cancelled"
            self.get_logger().warn('CLEANING homing CANCELLED')
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = HomingServiceServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()