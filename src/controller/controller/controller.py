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
        self.subscription = self.create_subscription(BBoxArray, "mediapipe_ros/bboxes", self.listener_callback, 10)
        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)

        self.talker = self.create_publisher(Talk, "aquestalkpi_ros/talk", 10)

        self.light_sensor_sub = self.create_subscription(LightSensors, "light_sensors", self.sensor_callback, 10)

        self.timer = self.create_timer(0.01, self.timer_cb)

        self.twist = Twist()
        self.found = False

    def listener_callback(self, bboxarr):
        maxscore = 0
        xmin = 160
        xmax = 160
        self.found = False
        for bbox in bboxarr.bboxes:
            if bbox.name == "person":
                self.found = True
                if bbox.score > maxscore:
                    maxscore = bbox.score
                    xmin = bbox.xmin
                    xmax = bbox.xmax

        center = (xmin + xmax) / 2
        if center < 100:
            self.twist.angular.z = 0.1  # * j
        elif center > 220:
            self.twist.angular.z = -0.1  # * j
        else:
            self.twist.angular.z = 0.0  # * j

    def sensor_callback(self, msg):
        d = clamp((msg.forward_r + msg.forward_l) / 2, 1, 500)

        if self.found:
            if d >= 100:  # self.twist.angular.z == 0.0 or
                self.twist.linear.x = 0.0
            elif 50 <= d and d < 100:
                self.twist.linear.x = 1 / d
            else:
                self.twist.linear.x = 0.02
        else:
            self.twist.linear.x = 0.0
            # self.twist.angular.z = 0.2

    def timer_cb(self):
        self.publisher.publish(self.twist)


def main():
    rclpy.init()
    node = Controller()
    rclpy.spin(node)
    rclpy.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
