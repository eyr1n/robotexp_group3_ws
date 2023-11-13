import rclpy
from aquestalkpi_ros_msgs.msg import Talk
from geometry_msgs.msg import Twist
from raspimouse_msgs.msg import LightSensors
from rclpy.node import Node

from mediapipe_ros_msgs.msg import BBoxArray

IMAGE_WIDTH = 320


def clamp(x, low=-float("inf"), high=float("inf")):
    return min(max(x, low), high)


class Controller(Node):
    def __init__(self):
        super().__init__("controller")

        # Subscription
        self.bboxes_sub = self.create_subscription(BBoxArray, "mediapipe_ros/bboxes", self.bboxes_sub_cb, 10)
        self.light_sensors_sub = self.create_subscription(LightSensors, "light_sensors", self.light_sensors_sub_cb, 10)

        # Publisher
        self.twist_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.talk_pub = self.create_publisher(Talk, "aquestalkpi_ros/talk", 10)

        # Timer
        self.timer = self.create_timer(0.01, self.timer_cb)  # 10ms

        self.twist_msg = Twist()
        self.found = False

    def bboxes_sub_cb(self, msg):
        self.found = False

        bboxes = sorted(
            filter(lambda x: x.name == "person", msg.bboxes),
            key=lambda x: x.score,
            reverse=True,
        )

        if bboxes:
            self.found = True

            center = (bboxes[0].xmin + bboxes[0].xmax) / 2  # 中心
            threshold = 60  # 閾値
            angular_vel = 0.1  # 角速度

            if center < IMAGE_WIDTH / 2 - threshold:
                self.twist_msg.angular.z = angular_vel
            elif center > IMAGE_WIDTH / 2 + threshold:
                self.twist_msg.angular.z = -angular_vel
            else:
                self.twist_msg.angular.z = 0.0

    def light_sensors_sub_cb(self, msg):
        d = clamp((msg.forward_r + msg.forward_l) / 2, low=1)

        if self.found:
            if d >= 100:
                self.twist_msg.linear.x = 0.0
            elif 50 <= d and d < 100:
                self.twist_msg.linear.x = 1 / d
            else:
                self.twist_msg.linear.x = 0.02
        else:
            self.twist_msg.linear.x = 0.0

    def timer_cb(self):
        self.twist_pub.publish(self.twist_msg)


def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
