from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from motion_control.sensor_data import *
from motion_control.alert_publisher import AlertPublisher

import math
import time

class PointsSubscriber(Node):

    def __init__(self, delay, precision, threshold):
        super().__init__('points_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            "/scan",
            self.listener_callback,
            10
            )
        self.subscription  # prevent unused variable warning
        self.prev_t = time.time_ns()
        self.alert_pub = AlertPublisher()
        self.delay = delay
        self.precision = precision
        self.threshold = threshold

    def listener_callback(self, msg):
        if time.time_ns() - self.prev_t <= self.delay:
            time.sleep(0.02)
            return
        self.prev_t = time.time_ns()
        print(len(msg.intensities))
        print(len(msg.ranges))
        for i in range(len(msg.ranges)):
            print(f'd : {msg.ranges[i]}, {self.precision}, i: {msg.intensities[i]}')
            if 0.07 <= msg.ranges[i] <= self.precision and msg.intensities[i] >= self.threshold:
                angle_lidar_point = msg.angle_min+msg.angle_increment*i
                angleAim_repRobot = SensorData.pos_angle - angle_lidar_point

                R = int(1000*msg.ranges[i]) + 1

                x_prime = R*math.cos(angleAim_repRobot)
                y_prime = R*math.sin(angleAim_repRobot)

                x_aim = SensorData.pos_x + x_prime
                y_aim = SensorData.pos_y + y_prime

                if(0 <= x_aim <= 3000 and 0 <= y_aim <=2000):
                    self.alert_pub.alert()
                    print("=================================")
                    return

        self.alert_pub.stop_alert()
        print("=====================================")