import rclpy
from aquestalkpi_ros_msgs.msg import Talk
from geometry_msgs.msg import Twist
from raspimouse_msgs.msg import LightSensors
from rclpy.node import Node

from mediapipe_ros_msgs.msg import BBoxArray


def clamp(x, l, h):
    return min(max(x, l), h)


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

        self.twist = Twist()
        self.found = False

    def bboxes_sub_cb(self, msg):
        maxscore = 0
        xmin = 160
        xmax = 160
        self.found = False
        for bbox in msg.bboxes:
            if bbox.name == "person":
                self.found = True
                if bbox.score > maxscore:
                    maxscore = bbox.score
                    xmin = bbox.xmin
                    xmax = bbox.xmax

        center = (xmin + xmax) / 2
        if center < 100:
            self.twist.angular.z = 0.1
        elif center > 220:
            self.twist.angular.z = -0.1
        else:
            self.twist.angular.z = 0.0

    def light_sensors_sub_cb(self, msg):
        d = clamp((msg.forward_r + msg.forward_l) / 2, 1, 500)

        if self.found:
            if d >= 100:
                self.twist.linear.x = 0.0
            elif 50 <= d and d < 100:
                self.twist.linear.x = 1 / d
            else:
                self.twist.linear.x = 0.02
        else:
            self.twist.linear.x = 0.0

    def timer_cb(self):
        self.twist_pub.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
