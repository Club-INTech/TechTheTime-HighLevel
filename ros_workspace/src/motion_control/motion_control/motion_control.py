from time import sleep
import rclpy
import sys
from motion_control.motion_subscriber import MotionSubscriber
from motion_control.points_subscriber import PointsSubscriber
# from motion_control.alert_publisher import AlertPublisher

def main(args=None):
    rclpy.init(args=args)
    # _callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
    executor = rclpy.executors.MultiThreadedExecutor()
    motion_subscriber = MotionSubscriber()
    points_subscriber = PointsSubscriber()

    executor.add_node(motion_subscriber)
    executor.add_node(points_subscriber)

    executor.spin()

    # rclpy.spin(points_subscriber)

    motion_subscriber.destroy_node()
    points_subscriber.destroy_node()
    # alert_publisher = AlertPublisher()
    # sleep(3)
    # alert_publisher.alert()
    # sleep(5)    
    # alert_publisher.stop_alert()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()