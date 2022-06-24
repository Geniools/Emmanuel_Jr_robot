#!/bin/usr/python3

import rclpy
from rclpy.node import Node
import time
import smbus
from imusensor.MPU9250 import MPU9250
from imusensor.filters import madgwick


class EmmanuelIMU(Node):
    def __init__(self):
        super().__init__("Emmanuel_IMU")
        address = 0x68
        bus = smbus.SMBus(1)
        self.sensorfusion = madgwick.Madgwick(0.5)
        self.imu = MPU9250.MPU9250(bus, address)
        self.imu.begin()
        # self.imu.caliberateGyro()
        # self.imu.caliberateAccelerometer()
        # # or load your own caliberation file
        # self.imu.loadCalibDataFromFile("/home/pi/calib_real_bolder.json")

        self.create_timer(0.1, self.updateSensorInfo)
        self.create_timer(1, self.updateSensorInfo)

    def updateSensorInfo(self):
        self.get_logger().info("Update sensor")
        self.imu.readSensor()
        currTime = time.time()
        for i in range(10):
            newTime = time.time()
            dt = newTime - currTime
            currTime = newTime

            self.sensorfusion.updateRollPitchYaw(self.imu.AccelVals[0], self.imu.AccelVals[1], self.imu.AccelVals[2],
                                                 self.imu.GyroVals[0],
                                                 self.imu.GyroVals[1], self.imu.GyroVals[2], self.imu.MagVals[0],
                                                 self.imu.MagVals[1],
                                                 self.imu.MagVals[2], dt)

    def printValues(self):
        self.get_logger().info(
            f"roll: {self.sensorfusion.roll}, pitch: {self.sensorfusion.pitch}, yaw: {self.sensorfusion.yaw}")
        self.get_logger().info(
            f"roll: {self.imu.roll}, pitch: {self.imu.pitch}, yaw: {self.imu.yaw}")


def main(args=None):
    rclpy.init(args=args)
    imuNode = EmmanuelIMU()

    try:
        rclpy.spin(imuNode)
    except KeyboardInterrupt:
        # Prevent error in case of "Ctrl+C"
        print("Closing because of keyboard interrupt")
    except Exception as e:
        print("An exception occurred: " + e)

    imuNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
