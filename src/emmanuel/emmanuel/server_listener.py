#!/bin/usr/python3
import rclpy
from rclpy.node import Node


class EmmanuelGoalExecutor(None):
    def __int__(self):
        super().__init__("EmmanuelServerListenerNode")


def main(args=None):
    rclpy.init(args=args)
    listener = EmmanuelGoalExecutor()

    try:
        rclpy.spin(listener)
    except KeyboardInterrupt:
        # Prevent error in case of "Ctrl+C"
        print("Closing because of keyboard interrupt")
    except Exception as e:
        print("An exception occurred: " + e)

    listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
