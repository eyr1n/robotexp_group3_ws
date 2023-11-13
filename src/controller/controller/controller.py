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
        self.timer = self.create_timer(0.01, self.timer_cb)

        self.persons = []
        self.distance = 1
        self.twist_msg = Twist()

    def bboxes_sub_cb(self, msg):
        self.persons = sorted(
            filter(lambda x: x.name == "person", msg.bboxes),
            key=lambda x: x.score,
            reverse=True,
        )

    def light_sensors_sub_cb(self, msg):
        # distance >= 1, 距離が短くなると増大
        self.distance = clamp((msg.forward_r + msg.forward_l) / 2, low=1)

    def timer_cb(self):  # 10msおきに呼ばれる関数
        if self.persons:  # personが見つかったら
            bbox = self.persons[0]
            center = (bbox.xmin + bbox.xmax) / 2  # 中心
            threshold = 60  # 閾値
            angular_vel = 0.1  # 角速度

            # 旋回
            if center < IMAGE_WIDTH / 2 - threshold:
                self.twist_msg.angular.z = angular_vel
            elif center > IMAGE_WIDTH / 2 + threshold:
                self.twist_msg.angular.z = -angular_vel
            else:
                self.twist_msg.angular.z = 0.0

            # 直進
            if self.distance >= 100:
                self.twist_msg.linear.x = 0.0
            elif 50 <= self.distance and self.distance < 100:
                self.twist_msg.linear.x = 1 / self.distance
            else:
                self.twist_msg.linear.x = 0.02
        else:
            self.twist_msg.linear.x = 0.0

        self.twist_pub.publish(self.twist_msg)


def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
