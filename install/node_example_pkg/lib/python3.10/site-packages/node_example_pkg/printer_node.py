
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyPrintingNode (Node): # MODIFY NAME
    def __init__(self):
        super().__init__("printer_node")
        self.get_logger().info("Hello from my node!")#This line lets you print to the terminal
	

def main(args=None):
  rclpy.init(args=args) 
  node = MyPrintingNode () # MODIFY NAME 
  rclpy.spin(node) 
  rclpy.shutdown()  