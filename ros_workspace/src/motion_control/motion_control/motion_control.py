import rclpy
from alert.alert_publisher import AlertPublisher

if __name__ == "__main__":
    rclpy.init(args=None)
    alert_publisher = AlertPublisher()
    while(True):
        alert = bool(input())
        if alert:
            alert_publisher.alert()
        else:
            alert_publisher.stop_alert()

    alert_publisher.destroy_node()
    rclpy.shutdown()

