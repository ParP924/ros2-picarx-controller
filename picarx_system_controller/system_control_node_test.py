#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import picarx

class PicarX(Node):
    def __init__(self):
        super().__init__('picarx_system_controller2')
        #self.picarx = PicarX()
        self.create_timer(1, self.timer_callback)
        self.get_logger().info('PicarX System Controller Node has been started')

    def timer_callback(self):
        #self.picarx.update()
        self.get_logger().info('PicarX System Controller Node is running')
        

def main(args=None):
    rclpy.init(args=args)
    picarx = PicarX()
    rclpy.spin(picarx)
    #picarx.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()