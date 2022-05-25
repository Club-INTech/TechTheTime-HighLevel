import multiprocessing
from time import sleep
import rclpy
import sys
import yaml
from motion_control.motion_subscriber import MotionSubscriber
from motion_control.points_subscriber import PointsSubscriber
from motion_control.const import DEFAULT_SCAN_PROCESSING_DELAY_NS, DEFAULT_STOP_PRECISION_MM


def main(args=None):

    config_file = sys.argv[1]

    team = "yellow"
    robot = "slave"

    delay = DEFAULT_SCAN_PROCESSING_DELAY_NS
    precision = DEFAULT_STOP_PRECISION_MM

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


    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=multiprocessing.cpu_count() or 2)
    motion_subscriber = MotionSubscriber(team, robot)
    points_subscriber = PointsSubscriber(delay, precision)

    executor.add_node(motion_subscriber)
    executor.add_node(points_subscriber)

    executor.spin()

    motion_subscriber.destroy_node()
    points_subscriber.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()