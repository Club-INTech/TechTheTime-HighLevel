from rclpy.node import Node

from sensor_msgs.msg import LaserScan

import math

class PointsSubscriber(Node):

    def __init__(self):
        super().__init__('points_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            "/scan",
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.object_in_game_area = []

    def listener_callback(self, msg):
        obj_groupes = self.__segmentation_groupe_point(msg)
        self.object_in_game_area = self.__discrimination(msg, obj_groupes)
        self.get_logger().info('==== Begin =====')
        for k in range(len(self.object_in_game_area)):
            self.get_logger().info('I see : x = %d et y = %d' % (self.object_in_game_area[k][0], self.object_in_game_area[k][1]))
        self.get_logger().info('==== END =====')


    def __segmentation_groupe_point(self, msg):
        L = len(msg.ranges)
        obj_groupes = []
        for i in range(L):
            nb_groupes = len(obj_groupes)
            if(nb_groupes==0):
                obj_groupes.append([[msg.ranges[i],i]])
            else:
                last_groupe_nb_points = len(obj_groupes[nb_groupes-1])
                #if(msg.ranges[i-1]-0.1<=msg.ranges[i]<=msg.ranges[i-1]+0.1)
                if(msg.ranges[i-1]-0.2<=msg.ranges[i]<=msg.ranges[i-1]+0.2):
                    obj_groupes[nb_groupes-1].append([msg.ranges[i],i])
                else:
                    obj_groupes.append([[msg.ranges[i],i]])
                #elif(i!=obj_groupes[nb_groupes-1][last_groupe_nb_points-1][1]+1):
        return obj_groupes

    def __representant(self, obj_groupes):
        rpz_groupe = []
        for k in range(len(obj_groupes)):
            l = len(obj_groupes[k])
            m = l//2
            rpz_groupe.append(obj_groupes[k][m])
        return rpz_groupe

    def __discrimination(self, msg, obj_groupes):
        x_robot = 400 #RobotMotion.x
        y_robot = 1000 #RobotMotion.y
        angle_robot = 0 #RobotMotion.angle
        rpz_groupe = self.__representant(obj_groupes)

        object_in_game_area = []

        for k in range(len(rpz_groupe)):
        
            angle_lidar_point = msg.angle_min+msg.angle_increment*rpz_groupe[k][1]
            R = rpz_groupe[k][0]*1000 # Conversion m from LIDAR to mm because all value are in mm in the code
            
            angleAim_repRobot = angle_robot - angle_lidar_point

            x_prime = R*math.cos(angleAim_repRobot)
            y_prime = R*math.sin(angleAim_repRobot)

            x_aim = x_robot + x_prime
            y_aim = y_robot + y_prime

            if(0 <= x_aim <= 3000 and 0 <= y_aim <=2000):
                if(k!=0 and k!=len(rpz_groupe)-1):
                    object_in_game_area.append([x_aim,y_aim])
        return object_in_game_area