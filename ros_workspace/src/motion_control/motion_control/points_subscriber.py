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
        for i in range(len(msg.ranges)):
            print(f'd : {msg.ranges[i]}, {self.precision}, i: {msg.intensities[i]}')
            if 0.07 <= msg.ranges[i] <= self.precision and msg.intensities[i] >= self.threshold:
                self.alert_pub.alert()
                return
        self.alert_pub.stop_alert()
    #     obj_groupes = self.__segmentation_groupe_point(msg)
    #     SensorData.object_in_game_area = self.__discrimination(msg, obj_groupes)
    #     for k in range(len(SensorData.object_in_game_area)):
    #         self.get_logger().info('[motion_control] I see : x = %d et y = %d' % (SensorData.object_in_game_area[k][0], SensorData.object_in_game_area[k][1]))
    #     for obj in SensorData.object_in_game_area:
    #         if (SensorData.pos_x - obj[0])**2 + (SensorData.pos_y - obj[1]) ** 2 <= self.precision**2:
    #             self.alert_pub.alert()
    #             return
    #     self.alert_pub.stop_alert()


    # def __segmentation_groupe_point(self, msg):
    #     L = len(msg.ranges)
    #     obj_groupes = []
    #     for i in range(L):
    #         nb_groupes = len(obj_groupes)
    #         if(nb_groupes==0):
    #             obj_groupes.append([[msg.ranges[i],i]])
    #         else:
    #             last_groupe_nb_points = len(obj_groupes[nb_groupes-1])
    #             #if(msg.ranges[i-1]-0.1<=msg.ranges[i]<=msg.ranges[i-1]+0.1)
    #             if(msg.ranges[i-1]-0.2<=msg.ranges[i]<=msg.ranges[i-1]+0.2):
    #                 obj_groupes[nb_groupes-1].append([msg.ranges[i],i])
    #             else:
    #                 obj_groupes.append([[msg.ranges[i],i]])
    #             #elif(i!=obj_groupes[nb_groupes-1][last_groupe_nb_points-1][1]+1):
    #     return obj_groupes

    # def __representant(self, obj_groupes):
    #     rpz_groupe = []
    #     for k in range(len(obj_groupes)):
    #         l = len(obj_groupes[k])
    #         m = l//2
    #         rpz_groupe.append(obj_groupes[k][m])
    #     return rpz_groupe

    # def __discrimination(self, msg, obj_groupes):
    #     x_robot = SensorData.pos_x
    #     y_robot = SensorData.pos_y
    #     angle_robot = SensorData.pos_angle
    #     rpz_groupe = self.__representant(obj_groupes)

    #     object_in_game_area = []

    #     for k in range(len(rpz_groupe)):
        
    #         angle_lidar_point = msg.angle_min+msg.angle_increment*rpz_groupe[k][1]
    #         R = rpz_groupe[k][0]*1000 # Conversion m from LIDAR to mm because all value are in mm in the code
            
    #         angleAim_repRobot = angle_robot - angle_lidar_point

    #         x_prime = R*math.cos(angleAim_repRobot)
    #         y_prime = R*math.sin(angleAim_repRobot)

    #         x_aim = x_robot + x_prime
    #         y_aim = y_robot + y_prime

    #         if(0 <= x_aim <= 3000 and 0 <= y_aim <=2000):
    #             if(k!=0 and k!=len(rpz_groupe)-1):
    #                 object_in_game_area.append([x_aim,y_aim])
    #     return object_in_game_area