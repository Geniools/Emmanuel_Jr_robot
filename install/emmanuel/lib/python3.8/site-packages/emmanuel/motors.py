#!/bin/usr/python3

import math
import time
from time import sleep

import RPi.GPIO as gp
from geometry_msgs.msg import Twist
from rclpy.node import Node
import rclpy
from gpiozero import Servo

from .encoder import Encoder


def clearGPIO():
    # Clearing the GPIO
    gp.cleanup()


class EmmanuelMotionMotors(Node):
    def __init__(self):
        # Setting up the node
        super().__init__("emmanuelJr_motors")
        self.get_logger().info("EmmanuelJr_motors node started")

        # Setting up the Servo motors
        self.servo1 = Servo(12)
        self.servo2 = Servo(13)
        self.servo3 = Servo(18)

        # Setting up pins for the motors (left motor)
        self.F_P1 = 24
        self.F_P2 = 23
        self.F_P3 = 27
        self.F_P4 = 17
        self.F_ENA = 25
        self.F_ENB = 22

        # Setting up pins for the motors (right motor)
        self.S_P1 = 16
        self.S_P2 = 20
        self.S_P3 = 5
        self.S_P4 = 6
        self.S_ENA = 21
        self.S_ENB = 26

        # Setting up GPIO and initializing first the motors
        gp.setmode(gp.BCM)
        gp.setup(self.F_P1, gp.OUT)
        gp.setup(self.F_P2, gp.OUT)
        gp.setup(self.F_P3, gp.OUT)
        gp.setup(self.F_P4, gp.OUT)

        gp.setup(self.F_ENA, gp.OUT)
        gp.setup(self.F_ENB, gp.OUT)

        # Setting up GPIO and initializing the second motor
        gp.setup(self.S_P1, gp.OUT)
        gp.setup(self.S_P2, gp.OUT)
        gp.setup(self.S_P3, gp.OUT)
        gp.setup(self.S_P4, gp.OUT)

        gp.setup(self.S_ENA, gp.OUT)
        gp.setup(self.S_ENB, gp.OUT)

        # Giving the frequency to the PWM pins
        self.f_LM = gp.PWM(self.F_ENA, 100)
        self.f_RM = gp.PWM(self.F_ENB, 100)

        self.s_LM = gp.PWM(self.S_ENA, 100)
        self.s_RM = gp.PWM(self.S_ENB, 100)

        # Starting the PWM pi
        self.f_LM.start(0)
        self.f_RM.start(0)

        self.s_LM.start(0)
        self.s_RM.start(0)

        # Setting up the encoders
        self.F_leftEncoder = Encoder(self.F_P1, self.F_P2, self.showEncoderData)
        self.F_rightEncoder = Encoder(self.F_P3, self.F_P4, self.showEncoderData)
        self.S_leftEncoder = Encoder(self.S_P1, self.S_P2, self.showEncoderData)
        self.S_rightEncoder = Encoder(self.S_P3, self.S_P4, self.showEncoderData)

        # Subscribing to the topic "cmd_vel" (getting linear and angular velocityu)
        self.create_subscription(Twist, "cmd_vel", self.callback_received_coordinates, 10)

    def showEncoderData(self, data):
        # Print the data received to the console
        self.get_logger().info("Encoder data: {}".format(data))

    def callback_received_coordinates(self, msg):
        # Getting the linear and angular velocity
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z
        self.f_LM.ChangeDutyCycle(75)
        self.f_RM.ChangeDutyCycle(75)

        self.s_LM.ChangeDutyCycle(75)
        self.s_RM.ChangeDutyCycle(75)

        # Checking if the linear velocity is positive or negative
        if linear_velocity > 0:
            self.moveForward()
        elif linear_velocity < 0:
            self.moveBackward()
        else:
            self.stopRobot()

        # # Converting the linear and angular velocity to the motor speeds
        # left_motor_speed = linear_velocity - angular_velocity * self.wheelbase / 2
        # right_motor_speed = linear_velocity + angular_velocity * self.wheelbase / 2
        #
        # # Converting the motor speeds to the PWM values
        # left_motor_speed_pwm = self.convert_velocity_to_pwm(left_motor_speed)
        # right_motor_speed_pwm = self.convert_velocity_to_pwm(right_motor_speed)
        #
        # # Setting the PWM values to the motors
        # self.set_motor_speed(left_motor_speed_pwm, right_motor_speed_pwm)

    def moveForward(self):
        self.gp.output(self.F_P1, True)
        self.gp.output(self.F_P2, False)

        self.gp.output(self.F_P3, True)
        self.gp.output(self.F_P4, False)

    def moveBackward(self):
        self.gp.output(self.F_P1, False)
        self.gp.output(self.F_P2, True)

        self.gp.output(self.F_P3, False)
        self.gp.output(self.F_P4, True)

    def stopRobot(self):
        self.gp.output(self.F_P1, False)
        self.gp.output(self.F_P2, False)

        self.gp.output(self.F_P3, False)
        self.gp.output(self.F_P4, False)


def main(args=None):
    rclpy.init(args=args)

    motorNode = EmmanuelMotionMotors()

    rclpy.spin(motorNode)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    clearGPIO()
    motorNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
