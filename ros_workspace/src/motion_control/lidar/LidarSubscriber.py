import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan

import math

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            "scan",
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        obj_groupes = []
        continuite = 1
        for i in range(len(msg.ranges)):
            if(msg.ranges[i]<=2.5):
                obj_groupes.append([msg.ranges[i],i])
        #self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

def segmentation_groupe_point(msg):
    L = len(msg.ranges)
    obj_groupes = []
    for i in range(L):
        if(msg.ranges[i]<=2):
            nb_groupes = len(obj_groupes)
            last_groupe_nb_points = len(obj_groupes[nb_groupes])
            if(nb_groupes!=0):
                if(i==obj_groupes[nb_groupes][last_groupe_nb_points][1]+1):
                    obj_groupes[nb_groupes].append([msg.ranges[i],i])
                elif(i!=obj_groupes[nb_groupes][last_groupe_nb_points][1]+1):
                    obj_groupes.append([[msg.ranges[i],i]])
            else:
                obj_groupes.append([[msg.ranges[i],i]])
    return obj_groupes

def representant(obj_groupes):
    rpz_groupe = []
    for k in range(obj_groupes):
        l = obj_groupes[k]
        m = l//2
        rpz_groupe.append(obj_groupes[k][m])
    return rpz_groupe

def discrimination(msg,obj_groupes):
    x_robot = RobotMotion.x
    y_robot = RobotMotion.y
    angle_robot = RobotMotion.angle
    rpz_groupe = representant(obj_groupes)

    for k in range(len(rpz_groupe)):
    
        angle_lidar_point = msg.angle_min+msg.angle_increment*rpz_groupe[k][1]
        R = rpz_groupe[k][0]

        angleAim_repRobot = angle_robot + angle_lidar_point

        x_prime = R*math.cos(angleAim_repRobot)
        y_prime = R*math.sin(angleAim_repRobot)

        x_aim = x_robot + x_prime
        y_aim = y_robot + y_prime