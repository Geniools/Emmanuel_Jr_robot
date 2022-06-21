#!/bin/usr/python3

from gpiozero import Servo
import rclpy
from rclpy.node import Node
from time import sleep
from std_msgs.msg import String


class EmmanuelServoMotor(Node):
    def __init__(self):
        super().__init__("EmmanuelServoMotor")
        self.get_logger().info("EmmanuelServoMotor node started")

        # Setting up the Servo motors
        self.head = Servo(13)
        self.servo1 = Servo(12)
        self.servo2 = Servo(18)

        # Subscribe to node receiving commands for the front arms and head
        self.create_subscription(String, "servo_command", self.servo_command_callback, 2)

    def servo_command_callback(self, msg):
        command = msg.data
        self.get_logger().info(f"Message received: {command}")

        if command == "right":
            self.moveHeadRight()
        if command == "left":
            self.moveHeadLeft()
        if command == "hmid":
            self.headMid()

        if command == "up":
            self.liftArms()
        if command == "mid":
            pass

    def moveHeadRight(self):
        self.head.max()

    def headMid(self):
        self.head.mid()

    def moveHeadLeft(self):
        self.head.min()

    def liftArms(self):
        for i in range(0, 10):
            self.servo1.value = i / 10
            sleep(0.5)

    def midArms(self):
        self.servo1.mid()
        self.servo2.mid()


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
