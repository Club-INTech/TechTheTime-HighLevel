from rclpy.node import Node
from motion_control.sensor_data import *

from motion_msg_srv.msg import Motion

class MotionSubscriber(Node):
    
    def __init__(self):
        super().__init__("motion_subscriber_python")
        self.subscription = self.create_subscription(
            Motion,
            'motion',
            self.listener_callback,
            10
        )
        self.subscription
    
    def listener_callback(self, msg):
        SensorData.atomic_move(SensorData, msg.left_ticks, msg.right_ticks)