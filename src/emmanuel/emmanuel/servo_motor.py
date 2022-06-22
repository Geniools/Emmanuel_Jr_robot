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
        self.head = Servo(18)
        self.servo1 = Servo(12)
        self.servo2 = Servo(13)

        # Subscribe to node receiving commands for the front arms and head
        self.create_subscription(String, "servo_command", self.servo_command_callback, 5)

    def servo_command_callback(self, msg):
        command = msg.data
        self.get_logger().info(f"Message received: {command}")

        if command == "right":
            self.get_logger().info("Moving head right")
            self.moveHeadRight()
        if command == "left":
            self.get_logger().info("Moving head left")
            self.moveHeadLeft()
        if command == "hmid":
            self.get_logger().info("Returning head initial position")
            self.headMid()

        if command == "up":
            self.liftArms()
        if command == "mid":
            self.midArms()

    def moveHeadRight(self):
        # self.head.min()
        for val in range(10):
            self.head.value = val / 10 * -1
            sleep(0.5)

    def headMid(self):
        self.head.mid()

    def moveHeadLeft(self):
        # self.head.max()
        for val in range(10):
            self.head.value = val / 10
            sleep(0.5)

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
