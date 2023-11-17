import threading

import rclpy
from aquestalkpi_ros_msgs.msg import Talk
from geometry_msgs.msg import Twist
from raspimouse_msgs.msg import LightSensors
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from mediapipe_ros_msgs.msg import BBoxArray

IMAGE_WIDTH = 320


def clamp(x, low=-float("inf"), high=float("inf")):
    return min(max(x, low), high)


class Controller(Node):
    def __init__(self):
        super().__init__("controller")

        # Subscription
        self.bboxes_sub = self.create_subscription(BBoxArray, "object_detector/bboxes", self.bboxes_sub_cb, 10)
        self.light_sensors_sub = self.create_subscription(LightSensors, "light_sensors", self.light_sensors_sub_cb, 10)

        # Publisher
        self.twist_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.talk_pub = self.create_publisher(Talk, "aquestalkpi_ros/talk", 10)

        # Timer
        timer_cb_group = MutuallyExclusiveCallbackGroup()
        self.timer = self.create_timer(0.01, self.timer_cb, callback_group=timer_cb_group)
        self.lock = threading.Lock()

        self.person = None
        self.distance = 1

    # 排他制御必須
    def bboxes_sub_cb(self, msg):
        persons = sorted(filter(lambda x: x.name == "person", msg.bboxes), key=lambda x: x.score, reverse=True)
        with self.lock:
            if persons:
                self.person = persons[0]
            else:
                self.person = None

    # 排他制御必須
    def light_sensors_sub_cb(self, msg):
        # distance >= 1, 距離が短くなると増大
        with self.lock:
            self.distance = clamp((msg.forward_r + msg.forward_l) / 2, low=1)

    def timer_cb(self):  # 10msおきに呼ばれる関数
        twist_msg = Twist()

        with self.lock:
            if self.person:  # personが見つかったら
                center = (self.person.xmin + self.person.xmax) / 2  # 中心
                threshold = 60  # 閾値
                angular_vel = 0.1  # 角速度

                # 旋回
                if center < IMAGE_WIDTH / 2 - threshold:
                    twist_msg.angular.z = angular_vel
                elif center > IMAGE_WIDTH / 2 + threshold:
                    twist_msg.angular.z = -angular_vel

                # 直進
                if self.distance < 50:
                    twist_msg.linear.x = 0.02
                elif self.distance < 100:
                    twist_msg.linear.x = 1 / self.distance

        self.twist_pub.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    executor = MultiThreadedExecutor()
    executor.add_node(controller)
    executor.spin()
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
