import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from adafruit_rplidar import RPLidar
from math import floor, pi
from time import time


class EmmanuelRPLidar(Node):
    def __init__(self):
        self.PORT = '/dev/ttyUSB0'
        self.lidar = RPLidar(None, self.PORT)
        self.scanRanges = [] * 360

        # Defining lidar attributes
        self.MAX_RANGE = 12
        self.SAMPLE_RATE = 8000
        self.ANGLE_MIN = 0 * (pi / 180)
        self.ANGLE_MAX = 359 * (pi / 180)
        self.lastTime = time()

        # Defining publisher
        self.publisher = self.create_publisher(LaserScan, 'scan', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def updateScanRanges(self):
        for scan in self.lidar.iter_scans():
            for (_, angle, distance) in scan:
                self.scanRanges[min([359, floor(angle)])] = distance

    def timer_callback(self):
        # Update scan ranges
        self.updateScanRanges()
        # Initialize scan message
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser_frame'

        scan.angle_min = self.ANGLE_MIN
        scan.angle_max = self.ANGLE_MAX

        scan.angle_increment = (self.ANGLE_MAX - self.ANGLE_MIN) / 360
        scan.time_increment = time() - self.lastTime

        scan.range_min = 0.12
        scan.range_max = self.MAX_RANGE
        scan.ranges = self.scanRanges
        self.publisher.publish(scan)

    def stopSpinning(self):
        self.lidar.stop_motor()

    def startSpinning(self):
        self.lidar.start_motor()
