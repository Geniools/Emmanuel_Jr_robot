#!/bin/usr/python3

import RPi.GPIO as gp
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from rclpy.node import Node
import rclpy
from math import fabs


class EmmanuelMotionMotors(Node):
    def __init__(self):
        # Setting up the node
        super().__init__("emmanuelJr_motors")
        self.get_logger().info("EmmanuelMotionMotors node started")

        # Setting up the light sensor
        self.lightSensorLeft = 19
        self.lightSensorRight = 4

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
        gp.setup(self.lightSensorLeft, gp.IN)
        gp.setup(self.lightSensorRight, gp.IN)

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

        # Subscribing to the topic "cmd_vel" (getting linear and angular velocity)
        self.create_subscription(Twist, "cmd_vel", self.callback_received_coordinates, 10)

        # Publishing to the topic "odom" (getting the odometry)
        self.odometryPublihser = self.create_publisher(Odometry, "odom/unfiltered", 10)

        # Check for prevening the increment of a pulse if the sensor returns the same value twice
        self.isPulseIncreasedLeft = False
        self.isPulseIncreasedRight = False
        # Robot moving speed
        self.velocityLeft = 0
        self.velocityRight = 0
        # Wheelbase - distance between the wheels
        self.wheelbase = 19.5 / 100  # Convert cm -> meters
        # Target velocities
        self.targetVelocityLeft = 0
        self.targetVelocityRight = 0
        # Distance between two pulses
        self.PULSE_WIDTH = 2.5 / 100  # distance between two pulses (in cm) / meters
        # Pulse counters (light sensors will count the pulses)
        self.leftPulseCounter = 0
        self.rightPulseCounter = 0

        # PID constants
        self.KP = 15
        self.KD = 10
        self.KI = 5
        # PID error variables
        self.previousSpeedErrorLeft = 0
        self.previousSpeedErrorRight = 0
        self.sumSpeedErrorLeft = 0
        self.sumSpeedErrorRight = 0

        # Publishing the odometry
        self.odometryTimer = self.create_timer(0.5, self.callback_publish_odometry)

        self.pulseCounterTimer = self.create_timer(0.01, self.updatePulses)

        self.updatePulseTimer = 1
        self.displayPulseCounter = self.create_timer(self.updatePulseTimer, self.updateVelocity)
        self.adjustSpeed = self.create_timer(0.001, self.correctSpeed)
        #
        # # Setting up the transform broadcaster
        # self.tf_broadcaster = TransformBroadcaster()

    def callback_publish_odometry(self):
        # Getting the odometry
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

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
        linearVelocity = msg.linear.x
        angularVelocity = msg.angular.z

        # Adjusting each wheel speed based on angular speed
        delta = self.wheelbase * angularVelocity
        self.targetVelocityRight = linearVelocity + delta
        self.targetVelocityLeft = linearVelocity - delta

        # Checking if the linear velocity is positive or negative
        # Based on it determine which direction the robot has to move
        if linearVelocity > 0:
            self.moveForward()
        elif linearVelocity < 0:
            self.moveBackwards()
        else:
            if angularVelocity > 0:
                self.turnLeft()
            elif angularVelocity < 0:
                self.turnRight()
            else:
                self.stopRobot()

        self.get_logger().info(f"Target: left: {self.targetVelocityLeft}, right: {self.targetVelocityRight}")

    def updatePulses(self):
        lightSensorValueLeft = gp.input(self.lightSensorLeft)
        lightSensorValueRight = gp.input(self.lightSensorRight)

        # Increasing pulse for the left wheel
        if lightSensorValueLeft == 0 and self.isPulseIncreasedLeft is False:
            self.leftPulseCounter += 1
            self.isPulseIncreasedLeft = True
        if lightSensorValueLeft == 1 and self.isPulseIncreasedLeft is True:
            self.isPulseIncreasedLeft = False

        # Increasing pulse for the right wheel
        if lightSensorValueRight == 0 and self.isPulseIncreasedRight is False:
            self.rightPulseCounter += 1
            self.isPulseIncreasedRight = True
        if lightSensorValueRight == 1 and self.isPulseIncreasedRight is True:
            self.isPulseIncreasedRight = False

    def updateVelocity(self):
        self.velocityLeft = self.leftPulseCounter * self.PULSE_WIDTH / self.updatePulseTimer
        self.velocityRight = self.rightPulseCounter * self.PULSE_WIDTH / self.updatePulseTimer

        # Reset the pulse counter after the previous counter has been printed
        # self.get_logger().info("Pulses right: {}".format(self.rightPulseCounter))
        # self.get_logger().info("Speed right: {}".format(self.velocityRight))
        self.rightPulseCounter = 0
        self.leftPulseCounter = 0

    def correctSpeed(self):
        error_left = fabs(self.targetVelocityLeft) - self.velocityLeft
        error_right = fabs(self.targetVelocityRight) - self.velocityRight

        targetPWM_left = (self.KP * error_left) + (self.KD * self.previousSpeedErrorLeft) + (
                self.KI * self.sumSpeedErrorLeft)
        targetPWM_right = (self.KP * error_right) + (self.KD * self.previousSpeedErrorRight) + (
                self.KI * self.sumSpeedErrorRight)

        targetPWM_left = max(min(100, targetPWM_left), 0)
        targetPWM_right = max(min(100, targetPWM_right), 0)

        # self.get_logger().info("Target PWM: left: {}, right: {}".format(targetPWM_left, targetPWM_right))

        self.changePWMRightMotor(targetPWM_right)
        self.changePWMLeftMotor(targetPWM_left)

        self.previousSpeedErrorLeft = error_left
        self.previousSpeedErrorRight = error_right

        self.sumSpeedErrorLeft += error_left
        self.sumSpeedErrorRight += error_right

    def changePWMLeftMotor(self, pwm):
        self.s_RM.ChangeDutyCycle(pwm)

    def changePWMRightMotor(self, pwm):
        self.s_LM.ChangeDutyCycle(pwm)

    def moveForward(self):
        gp.output(self.F_P1, True)
        gp.output(self.F_P2, False)

        gp.output(self.F_P3, False)
        gp.output(self.F_P4, True)

        gp.output(self.S_P1, True)
        gp.output(self.S_P2, False)

        gp.output(self.S_P3, False)
        gp.output(self.S_P4, True)

    def moveBackwards(self):
        gp.output(self.F_P1, False)
        gp.output(self.F_P2, True)

        gp.output(self.F_P3, True)
        gp.output(self.F_P4, False)

        gp.output(self.S_P1, False)
        gp.output(self.S_P2, True)

        gp.output(self.S_P3, True)
        gp.output(self.S_P4, False)

    def turnRight(self):
        gp.output(self.F_P1, True)
        gp.output(self.F_P2, False)

        gp.output(self.F_P3, False)
        gp.output(self.F_P4, True)

        gp.output(self.S_P1, False)
        gp.output(self.S_P2, True)

        gp.output(self.S_P3, True)
        gp.output(self.S_P4, False)

    def turnLeft(self):
        gp.output(self.F_P1, False)
        gp.output(self.F_P2, True)

        gp.output(self.F_P3, True)
        gp.output(self.F_P4, False)

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


def main(args=None):
    rclpy.init(args=args)
    motorNode = EmmanuelMotionMotors()

    try:
        rclpy.spin(motorNode)
    except KeyboardInterrupt:
        # Prevent error in case of "Ctrl+C"
        print("Closing because of keyboard interrupt")
    except Exception:
        print("An exception occurred" + str(Exception))

    motorNode.cleanup()
    motorNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
