import multiprocessing
from time import sleep
import rclpy
import sys
import yaml
from motion_control.motion_subscriber import MotionSubscriber
from motion_control.points_subscriber import PointsSubscriber
from motion_control.const import DEFAULT_SCAN_PROCESSING_DELAY_NS, DEFAULT_STOP_PRECISION_MM
import threading

def runner(node):
    rclpy.spin(node)


def main(args=None):

    team = "yellow"
    robot = "slave"

    delay = DEFAULT_SCAN_PROCESSING_DELAY_NS
    precision = DEFAULT_STOP_PRECISION_MM

    threshold = 50

    if len(sys.argv) >= 2:
        with open(sys.argv[1], 'r') as config:
            data = yaml.safe_load(config)
            if not data["team"] is None:
                team = data["team"]
            if not data["robot"] is None:
                robot = data["robot"]
            if not data["delay"] is None:
                delay = data["delay"]
            if not data["precision"] is None:
                precision = data["precision"]
            if not data["threshold"] is None:
                threshold = data["threshold"]


    rclpy.init(args=args)

    motion_subscriber = MotionSubscriber(team, robot)
    points_subscriber = PointsSubscriber(delay, precision, threshold)

    motion_thread = threading.Thread(target=runner, args=(motion_subscriber,))
    points_thread = threading.Thread(target=runner, args=(points_subscriber,))

    motion_thread.start()
    points_thread.start()

    motion_thread.join()
    points_thread.join()

    motion_subscriber.destroy_node()
    points_subscriber.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()