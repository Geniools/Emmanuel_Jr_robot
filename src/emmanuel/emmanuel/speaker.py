#!/bin/usr/python3
import rclpy
from rclpy.node import Node


class EmmanuelSpeaker(Node):
    def __int__(self):
        super().__init__("EmmanuelSpeakerNode")


def main(args=None):
    rclpy.init(args=args)
    speakerNode = EmmanuelSpeaker()

    try:
        rclpy.spin(speakerNode)
    except KeyboardInterrupt:
        # Prevent error in case of "Ctrl+C"
        print("Closing because of keyboard interrupt")
    except Exception as e:
        print("An exception occurred: " + e)

    speakerNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
