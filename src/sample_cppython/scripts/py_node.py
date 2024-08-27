"""
Name: Cameron Basara
Date: 3/11/2024
Purpose: Sample python node for a sample package to for copying (mixed cpp/python)
"""
#!/usr/bin/env python3

# Import libraries ....
import rclpy
from rclpy.node import Node

class pyNode(Node):
    def __init__(self):
        super().__init__("py_node")

def main(args=None):
    # Initialize our node, 
    rclpy.init(args=args)
    node = pyNode()

    try:
        # Publish etc.
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



