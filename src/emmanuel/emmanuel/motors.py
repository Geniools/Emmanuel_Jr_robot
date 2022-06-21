#!/bin/usr/python3

import math
import time
from time import sleep

import RPi.GPIO as gp
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from rclpy.node import Node
import rclpy
from gpiozero import Servo

from .encoder import Encoder


class EmmanuelMotionMotors(Node):
    def __init__(self):
        # Setting up the node
        super().__init__("emmanuelJr_motors")
        self.get_logger().info("EmmanuelJr_motors node started")

        # Setting up the light sensor
        self.lightSensor = 19

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

        # Setting up GPIO
        gp.setmode(gp.BCM)
        gp.setwarnings(False)

        # Setting up the light sensor
        gp.setup(self.lightSensor, gp.IN)

        # Initializing first the motors
        gp.setup(self.F_P1, gp.OUT)
        gp.setup(self.F_P2, gp.OUT)
        gp.setup(self.F_P3, gp.OUT)
        gp.setup(self.F_P4, gp.OUT)

        gp.output(self.F_P1, False)
        gp.output(self.F_P2, False)
        gp.output(self.F_P3, False)
        gp.output(self.F_P4, False)

        gp.setup(self.F_ENA, gp.OUT)
        gp.setup(self.F_ENB, gp.OUT)

        # Setting up GPIO and initializing the second motor
        gp.setup(self.S_P1, gp.OUT)
        gp.setup(self.S_P2, gp.OUT)
        gp.setup(self.S_P3, gp.OUT)
        gp.setup(self.S_P4, gp.OUT)

        gp.output(self.S_P1, False)
        gp.output(self.S_P2, False)
        gp.output(self.S_P3, False)
        gp.output(self.S_P4, False)

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

        # Subscribing to the topic "cmd_vel" (getting linear and angular velocity)
        self.create_subscription(Twist, "cmd_vel", self.callback_received_coordinates, 10)
        # Publishing to the topic "odom" (getting the odometry)
        self.create_publisher(Odometry, "odom/unfiltered", 10)

        # Publishing timer (how often it will be published)
        publishOdometryPeriod = 0.1
        displayEncoderDataPeriod = 0.01
        self.pulseCounter = 0
        self.previousPulseCounter = 0
        self.previousLightSensorValue = 0
        self.isPulseIncreased = False

        self.odometryTimer = self.create_timer(publishOdometryPeriod, self.callback_publish_odometry)
        self.pulseCounterTimer = self.create_timer(displayEncoderDataPeriod, self.getLightSensor)
        self.displayPulseCounter = self.create_timer(1, self.getPulseCounter)
        #
        # # Setting up the transform broadcaster
        # self.tf_broadcaster = TransformBroadcaster()

    def callback_publish_odometry(self):
        # Getting the odometry
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

    def showEncoderData(self, data):
        # Print the data received to the console
        self.get_logger().info("Encoder data: {}".format(data))

    def callback_received_coordinates(self, msg):
        self.get_logger().info("Received coordinates: {}".format(msg))

        gp.setup(self.F_P1, gp.OUT)
        gp.setup(self.F_P2, gp.OUT)
        gp.setup(self.F_P3, gp.OUT)
        gp.setup(self.F_P4, gp.OUT)

        gp.setup(self.S_P1, gp.OUT)
        gp.setup(self.S_P2, gp.OUT)
        gp.setup(self.S_P3, gp.OUT)
        gp.setup(self.S_P4, gp.OUT)

        # Getting the linear and angular velocity
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        # self.f_LM.ChangeDutyCycle(50)
        # self.f_RM.ChangeDutyCycle(50)

        # # Converting the linear and angular velocity to the motor speeds
        # left_motor_speed = linear_velocity - angular_velocity * self.wheelbase / 2
        # right_motor_speed = linear_velocity + angular_velocity * self.wheelbase / 2
        #
        # # Converting the motor speeds to the PWM values
        # left_motor_speed_pwm = self.convert_velocity_to_pwm(left_motor_speed)
        # right_motor_speed_pwm = self.convert_velocity_to_pwm(right_motor_speed)

        self.changePWMLeftMotor(75)
        self.changePWMRightMotor(75)

        # Checking if the linear velocity is positive or negative
        if linear_velocity > 0:
            self.moveForward()
        elif linear_velocity < 0:
            self.moveBackwards()
        else:
            self.stopRobot()

    def changePWMLeftMotor(self, pwm):
        self.s_LM.ChangeDutyCycle(pwm)

    def changePWMRightMotor(self, pwm):
        self.s_RM.ChangeDutyCycle(pwm)

    def moveForward(self):
        # Setup first motor go forward
        gp.output(self.F_P1, True)
        gp.output(self.F_P2, False)

        gp.output(self.F_P3, False)
        gp.output(self.F_P4, True)

        # Setup first motor go forward
        gp.output(self.S_P1, True)
        gp.output(self.S_P2, False)

        gp.output(self.S_P3, False)
        gp.output(self.S_P4, True)

    def moveBackwards(self):
        gp.output(self.F_P1, False)
        gp.output(self.F_P2, True)

        gp.output(self.F_P3, True)
        gp.output(self.F_P4, False)

        # Setup first motor go forward
        gp.output(self.S_P1, False)
        gp.output(self.S_P2, True)

        gp.output(self.S_P3, True)
        gp.output(self.S_P4, False)

    def turnRight(self):
        gp.output(self.F_P1, True)
        gp.output(self.F_P2, False)

        gp.output(self.F_P3, False)
        gp.output(self.F_P4, True)

        # Setup first motor go forward
        gp.output(self.S_P1, False)
        gp.output(self.S_P2, True)

        gp.output(self.S_P3, True)
        gp.output(self.S_P4, False)

    def turnLeft(self):
        gp.output(self.F_P1, False)
        gp.output(self.F_P2, True)

        gp.output(self.F_P3, True)
        gp.output(self.F_P4, False)

        # Setup first motor go forward
        gp.output(self.S_P1, True)
        gp.output(self.S_P2, False)

        gp.output(self.S_P3, False)
        gp.output(self.S_P4, True)

    def stopRobot(self):
        gp.output(self.F_P1, False)
        gp.output(self.F_P2, False)

        gp.output(self.F_P3, False)
        gp.output(self.F_P4, False)

        gp.output(self.S_P1, False)
        gp.output(self.S_P2, False)

        gp.output(self.S_P3, False)
        gp.output(self.S_P4, False)

    def cleanup(self):
        self.get_logger().info("Cleaning up...")
        self.f_LM.stop()
        self.f_RM.stop()

        self.s_LM.stop()
        self.s_RM.stop()

    def getLightSensor(self):
        lightSensorValue = gp.input(self.lightSensor)

        if lightSensorValue == 0 and self.isPulseIncreased is False:
            self.pulseCounter += 1
            self.isPulseIncreased = True
        if lightSensorValue == 1 and self.isPulseIncreased is True:
            self.isPulseIncreased = False

        self.get_logger().info("Light sensor: {}".format(lightSensorValue))
        return lightSensorValue

    def getPulseCounter(self):
        # self.previousPulseCounter = self.pulseCounter
        self.get_logger().info("Pulses per second: {}".format(self.pulseCounter))
        return self.pulseCounter


def main(args=None):
    rclpy.init(args=args)
    motorNode = EmmanuelMotionMotors()

    try:
        rclpy.spin(motorNode)

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)

        motorNode.cleanup()
        motorNode.destroy_node()
        rclpy.shutdown()

    except KeyboardInterrupt:
        motorNode.cleanup()
        motorNode.destroy_node()


if __name__ == '__main__':
    main()
