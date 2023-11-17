import random
import threading
import time
from enum import Enum, auto

import rclpy
from aquestalkpi_ros_msgs.msg import Talk
from geometry_msgs.msg import Twist
from raspimouse_msgs.msg import LightSensors
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from mediapipe_ros_msgs.msg import BBoxArray, GestureArray

IMAGE_WIDTH = 320


class State(Enum):
    FOUND = auto()
    LOST = auto()
    STOP = auto()
    RPS = auto()


def clamp(x, low=-float("inf"), high=float("inf")):
    return min(max(x, low), high)


class Controller(Node):
    def __init__(self):
        super().__init__("controller")

        # Subscription
        self.bboxes_sub = self.create_subscription(BBoxArray, "object_detector/bboxes", self.bboxes_sub_cb, 10)
        self.gestures_sub = self.create_subscription(
            GestureArray, "gesture_recognizer/gestures", self.gestures_sub_cb, 10
        )
        self.light_sensors_sub = self.create_subscription(LightSensors, "light_sensors", self.light_sensors_sub_cb, 10)

        # Publisher
        self.twist_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.talk_pub = self.create_publisher(Talk, "aquestalkpi_ros/talk", 10)

        # Timer
        timer_cb_group = MutuallyExclusiveCallbackGroup()
        self.timer = self.create_timer(0.01, self.timer_cb, callback_group=timer_cb_group)
        self.lock = threading.Lock()

        self.person = None
        self.gesture = None
        self.distance = 1

        self.prev_state = State.LOST
        self.state = State.LOST

    def bboxes_sub_cb(self, msg):
        persons = sorted(filter(lambda x: x.name == "person", msg.bboxes), key=lambda x: x.score, reverse=True)
        with self.lock:
            if persons:
                self.person = persons[0]
            else:
                self.person = None

    def gestures_sub_cb(self, msg):
        gestures = sorted(msg.gestures, key=lambda x: x.score, reverse=True)
        with self.lock:
            if gestures:
                self.gesture = gestures[0]
            else:
                self.gesture = None

    def light_sensors_sub_cb(self, msg):
        # distance >= 1, 距離が短くなると増大
        with self.lock:
            self.distance = clamp((msg.forward_r + msg.forward_l) / 2, low=1)

    def timer_cb(self):  # 10msおきに呼ばれる関数
        with self.lock:
            person_found = True if self.person else False
            if person_found:
                center = (self.person.xmin + self.person.xmax) / 2  # 中心
            distance = self.distance
            if self.gesture:
                if self.gesture.name == "Thumb_Up":
                    self.state = State.RPS

        twist_msg = Twist()
        talk_msg = Talk()

        if self.state == State.LOST:
            if self.state != self.prev_state:
                talk_msg.text = "どこにいるのー"
                self.talk_pub.publish(talk_msg)
                self.prev_state = self.state

            twist_msg.angular.z = 0.25
            if person_found:
                self.state = State.FOUND

        elif self.state == State.FOUND:
            if self.state != self.prev_state:
                talk_msg.text = "みつけたー"
                self.talk_pub.publish(talk_msg)
                self.prev_state = self.state

            if person_found:
                threshold = 60  # 閾値
                angular_vel = 0.15  # 角速度
                # 旋回
                if center < IMAGE_WIDTH / 2 - threshold:
                    twist_msg.angular.z = angular_vel
                elif center > IMAGE_WIDTH / 2 + threshold:
                    twist_msg.angular.z = -angular_vel
                # 直進
                if distance < 40:
                    twist_msg.linear.x = 0.05
                elif distance < 80:
                    twist_msg.linear.x = 1 / distance
                else:
                    self.state = State.STOP
            else:
                self.state = State.LOST

        elif self.state == State.STOP:
            if self.state != self.prev_state:
                talk_msg.text = "ぶつかるー"
                self.talk_pub.publish(talk_msg)
                self.prev_state = self.state

            if distance < 50:
                if person_found:
                    self.state = State.FOUND
                else:
                    self.state = State.LOST

        elif self.state == State.RPS:
            if self.state != self.prev_state:
                self.twist_pub.publish(twist_msg)
                self.rock_paper_scissors()
                self.prev_state = self.state

        self.twist_pub.publish(twist_msg)

    def rock_paper_scissors(self):
        msg = Talk()
        msg.text = "じゃんけんしよう"
        self.talk_pub.publish(msg)
        time.sleep(2)

        finger = random.randint(0, 2)  # 0:グー 1:チョキ 2:パー
        if finger == 0:
            finger_text = "グー"
        if finger == 1:
            finger_text = "チョキ"
        if finger == 2:
            finger_text = "パー"
        msg.text = "さいしょはグー、じゃん、けん、" + finger_text
        self.talk_pub.publish(msg)
        time.sleep(4)

        with self.lock:
            if self.gesture:
                if (
                    (finger == 0 and self.gesture.name == "Open_Palm")
                    or (finger == 1 and self.gesture.name == "Closed_Fist")
                    or (finger == 2 and self.gesture.name == "Victory")
                ):
                    msg.text = "まけたー"
                elif (
                    (finger == 0 and self.gesture.name == "Victory")
                    or (finger == 1 and self.gesture.name == "Open_Palm")
                    or (finger == 2 and self.gesture.name == "Closed_Fist")
                ):
                    msg.text = "かったー"
                else:
                    msg.text = "あいこだね"
            else:
                msg.text = "よくみえなかったよ"
                self.state = State.LOST

        self.talk_pub.publish(msg)
        time.sleep(3)
        self.state = State.LOST


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
