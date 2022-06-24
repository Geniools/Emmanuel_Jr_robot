#!/bin/usr/python3
import rclpy
from rclpy.node import Node


class EmmanuelCamera(Node):
    def __init__(self):
        super().__init__("EmmanuelCameraNode")


def main(args=None):
    rclpy.init(args=args)
    cameraNode = EmmanuelCamera()

    try:
        rclpy.spin(cameraNode)
    except KeyboardInterrupt:
        # Prevent error in case of "Ctrl+C"
        print("Closing because of keyboard interrupt")
    except Exception as e:
        print("An exception occurred: " + e)

    cameraNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
