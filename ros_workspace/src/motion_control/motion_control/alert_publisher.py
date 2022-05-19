import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool


class AlertPublisher(Node):

    def __init__(self):
        super().__init__('alert_publisher')
        self.publisher_ = self.create_publisher(Bool, 'alert', 10)
        self.__alert = False
        self.__prev_alert = False
    
    def alert(self):
        print("Alert")
        self.__prev_alert = self.__alert
        self.__alert = True
        if not self.__prev_alert:
            self.__publish_msg()

    def stop_alert(self):
        print("Stop alert")
        self.__prev_alert = self.__alert
        self.__alert = False
        if self.__prev_alert:
            self.__publish_msg()

    def __publish_msg(self):
        msg = Bool()
        msg.data = self.__alert
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: ' +  str(msg.data))