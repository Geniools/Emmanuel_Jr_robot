#!/bin/usr/python3

from gpiozero import Servo
import rclpy
from rclpy.node import Node


class EmmanuelServoMotor(Node):
    def __init__(self):
        super().__init__("EmmanuelServoMotor")
        self.get_logger().info("EmmanuelServoMotor node started")

        # Setting up the Servo motors
        self.servo1 = Servo(12)
        self.servo2 = Servo(13)
        self.servo3 = Servo(18)


def main(args=None):
    rclpy.init(args=args)
    motorNode = EmmanuelServoMotor()

    try:
        rclpy.spin(motorNode)

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)

        motorNode.destroy_node()
        rclpy.shutdown()

    except KeyboardInterrupt:
        motorNode.destroy_node()


if __name__ == '__main__':
    main()
