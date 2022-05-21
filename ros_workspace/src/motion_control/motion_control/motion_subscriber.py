from rclpy.node import Node
from motion_control.sensor_data import *

from motion_msg_srv.msg import Motion

class MotionSubscriber(Node):
    
    def __init__(self, team, robot):
        super().__init__("motion_subscriber_python")
        self.subscription = self.create_subscription(
            Motion,
            'motion',
            self.listener_callback,
            10
        )
        self.subscription

        if team == "yellow":
            if robot == "master":
                SensorData.pos_x = START_X_1A_YELLOW
                SensorData.pos_y = START_Y_1A_YELLOW
                SensorData.pos_angle_ = START_ANGLE_1A_YELLOW
                SensorData.pos_angle = START_ANGLE_1A_YELLOW
            elif robot == "slave":
                SensorData.pos_x = START_X_2A_YELLOW
                SensorData.pos_y = START_Y_2A_YELLOW
                SensorData.pos_angle_ = START_ANGLE_2A_YELLOW
                SensorData.pos_angle = START_ANGLE_2A_YELLOW
        elif team == "purple":
            if robot == "master":
                SensorData.pos_x = START_X_1A_PURPLE
                SensorData.pos_y = START_Y_1A_PURPLE
                SensorData.pos_angle_ = START_ANGLE_1A_PURPLE
                SensorData.pos_angle = START_ANGLE_1A_PURPLE
            elif robot == "slave":
                SensorData.pos_x = START_X_2A_PURPLE
                SensorData.pos_y = START_Y_2A_PURPLE
                SensorData.pos_angle_ = START_ANGLE_2A_PURPLE
                SensorData.pos_angle = START_ANGLE_2A_PURPLE
    
    def listener_callback(self, msg):
        SensorData.atomic_move(SensorData, msg.left_ticks, msg.right_ticks)