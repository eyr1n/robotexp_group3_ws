import rclpy
from aquestalkpi_ros_msgs.msg import Talk
from geometry_msgs.msg import Twist
from rclpy.node import Node

from nanodet_ros_msgs.msg import BBox, BBoxArray


class Controller(Node):
    def __init__(self):
        super().__init__("controller")
        self.subscription = self.create_subscription(BBoxArray, "nanodet_ros/bboxes", self.listener_callback, 10)
        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)

        self.talker = self.create_publisher(Talk, "aquestalkpi_ros/talk", 10)

    def listener_callback(self, bboxarr):
        maxscore = 0
        xmin = 160
        xmax = 160
        for bbox in bboxarr.bboxes:
            if bbox.name == "person":
                if bbox.score > maxscore:
                    maxscore = bbox.score
                    xmin = bbox.xmin
                    xmax = bbox.xmax

        center = (xmin + xmax) / 2
        twist = Twist()
        if center < 150:
            twist.angular.z = 0.3  # * j
        if center > 170:
            twist.angular.z = -0.3  # * j

        self.publisher.publish(twist)


def main():
    rclpy.init()
    node = Controller()
    rclpy.spin(node)
    rclpy.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
