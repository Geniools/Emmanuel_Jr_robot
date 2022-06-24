#!/bin/usr/python3
import time

import RPi.GPIO as gp
from geometry_msgs.msg import Twist, TransformStamped, TwistWithCovariance
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler
from rclpy.node import Node
import rclpy
from math import fabs, sin, cos


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

        self.prev_update_time = time.time()

        self.odom_trans = TransformStamped()
        self.odom_trans.header.frame_id = 'odom'
        self.odom_trans.child_frame_id = 'base_link'

        self.odometry = Odometry()
        self.odometry.header.frame_id = 'odom'
        self.odometry.child_frame_id = 'base_link'

        self.x = 0
        self.y = 0
        self.dth = 0

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
        self.PULSE_WIDTH = 2 / 100  # distance between two pulses (in cm) / meters

        # Pulse counters (light sensors will count the pulses)
        self.leftPulseCounter = 0
        self.rightPulseCounter = 0

        # PID constants
        self.KP = 4
        self.KD = 2
        self.KI = 1

        # PID error variables
        self.previousSpeedErrorLeft = 0
        self.previousSpeedErrorRight = 0
        self.sumSpeedErrorLeft = 0
        self.sumSpeedErrorRight = 0

        # Initial PWD
        self.targetPWMLeft = 40
        self.targetPWMRight = 40

        # Robot is moving
        self.isMoving = False

        self.pulseCounterTimer = self.create_timer(0.001, self.updatePulses)

        self.updatePulseTimer = 1
        self.create_timer(self.updatePulseTimer, self.updateVelocity)
        self.create_timer(0.3, self.correctSpeed)
        # Publishing the odometry
        self.odometryTimer = self.create_timer(self.updatePulseTimer, self.callback_publish_odometry)
        # self.create_timer(1, self.displayVelocity)

    def displayVelocity(self):
        self.get_logger().info(f"Actual speed: left: {self.velocityLeft}, right: {self.velocityRight}")

    def callback_publish_odometry(self):
        current_time = time.time()

        linearVelocity = (self.velocityLeft + self.velocityRight) / 2
        angularVelocity = (self.velocityRight - self.velocityLeft) / self.wheelbase

        dt = (current_time - self.prev_update_time)
        delta_th = angularVelocity * dt

        delta_x = linearVelocity * cos(delta_th) * dt
        delta_y = linearVelocity * sin(delta_th) * dt

        self.x += delta_x
        self.y += delta_y
        self.dth += delta_th

        self.odometry.pose.pose.position.x = self.x
        self.odometry.pose.pose.position.y = self.y
        self.odometry.pose.pose.position.z = 0.0

        quaternion = quaternion_from_euler(0, 0, self.dth)  # roll,pitch,yaw
        self.odometry.pose.pose.orientation.x = quaternion[0]
        self.odometry.pose.pose.orientation.y = quaternion[1]
        self.odometry.pose.pose.orientation.z = quaternion[2]
        self.odometry.pose.pose.orientation.w = quaternion[3]

        self.odometry.twist.twist.linear.x = linearVelocity * -1
        self.odometry.twist.twist.linear.y = 0.0
        self.odometry.twist.twist.linear.z = 0.0
        self.odometry.twist.twist.angular.x = 0.0
        self.odometry.twist.twist.angular.y = 0.0
        self.odometry.twist.twist.angular.z = angularVelocity * -1

        self.odometry.header.stamp = self.get_clock().now().to_msg()
        self.odometryPublihser.publish(self.odometry)

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
        delta = self.wheelbase * angularVelocity / 2
        self.targetVelocityRight = fabs(linearVelocity - delta)
        self.targetVelocityLeft = fabs(linearVelocity + delta)

        # Checking if the linear velocity is positive or negative
        # Based on it determine which direction the robot has to move
        self.targetPWMRight = 40
        self.targetPWMLeft = 40
        self.isMoving = True
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
                self.isMoving = False
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

        self.get_logger().info(f"Actual speed: left: {self.velocityLeft}, right: {self.velocityRight}")

        # Reset the pulse counter after the previous counter has been printed
        self.get_logger().info("Pulses left: {}, right: {}".format(self.rightPulseCounter, self.leftPulseCounter))
        self.rightPulseCounter = 0
        self.leftPulseCounter = 0

    def correctSpeed(self):
        if self.isMoving is False:
            return

        error_left = (fabs(self.targetVelocityLeft) - self.velocityLeft)
        error_right = (fabs(self.targetVelocityRight) - self.velocityRight)

        self.targetPWMLeft += (self.KP * error_left) + (self.KD * self.previousSpeedErrorLeft) + (
                self.KI * self.sumSpeedErrorLeft)
        self.targetPWMRight += (self.KP * error_right) + (self.KD * self.previousSpeedErrorRight) + (
                self.KI * self.sumSpeedErrorRight)

        targetPWM_left = max(min(100, self.targetPWMLeft), 0)
        targetPWM_right = max(min(100, self.targetPWMRight), 0)

        self.get_logger().info("Target PWM: left: {}, right: {}".format(targetPWM_left, targetPWM_right))

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

    def turnLeft(self):
        gp.output(self.F_P1, True)
        gp.output(self.F_P2, False)

        gp.output(self.F_P3, False)
        gp.output(self.F_P4, True)

        gp.output(self.S_P1, False)
        gp.output(self.S_P2, True)

        gp.output(self.S_P3, False)
        gp.output(self.S_P4, True)

    def turnRight(self):
        gp.output(self.F_P1, False)
        gp.output(self.F_P2, True)

        gp.output(self.F_P3, True)
        gp.output(self.F_P4, False)

        gp.output(self.S_P1, True)
        gp.output(self.S_P2, False)

        gp.output(self.S_P3, True)
        gp.output(self.S_P4, False)

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
    except Exception as e:
        print("An exception occurred: " + e)

    motorNode.cleanup()
    motorNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
