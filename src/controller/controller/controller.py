import random
import threading
import time

import rclpy
from aquestalkpi_ros_msgs.msg import Talk
from geometry_msgs.msg import Twist
from raspimouse_msgs.msg import LightSensors
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from mediapipe_ros_msgs.msg import BBoxArray, GestureArray

from .state_manager import StateManager

IMAGE_WIDTH = 320


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

        # state manager
        self.manager = StateManager()
        self.manager.add("init", self.init_cb)
        self.manager.add("lost_person", self.lost_person_cb)
        self.manager.add("find_person", self.find_person_cb)
        self.manager.add("wall", self.wall_cb)

        self.manager.change("init")
        self.manager.run()

        self.stopped = False

    def init_cb(self):
        msg = Talk()
        msg.text = "start"
        # self.talk_pub.publish(msg)

    def lost_person_cb(self):
        msg = Talk()
        msg.text = "どこにいるのー"
        self.talk_pub.publish(msg)

    def find_person_cb(self):
        msg = Talk()
        msg.text = "みつけたー"
        self.talk_pub.publish(msg)

    def wall_cb(self):
        msg = Talk()
        msg.text = "ぶつかるー"
        # self.talk_pub.publish(msg)

    # 排他制御必須
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

    # 排他制御必須
    def light_sensors_sub_cb(self, msg):
        # distance >= 1, 距離が短くなると増大
        with self.lock:
            self.distance = clamp((msg.forward_r + msg.forward_l) / 2, low=1)

    def timer_cb(self):  # 10msおきに呼ばれる関数
        twist_msg = Twist()

        with self.lock:
            person_found = True if self.person else False
            if person_found:
                center = (self.person.xmin + self.person.xmax) / 2  # 中心
            distance = self.distance

        if person_found:  # personが見つかったら
            if self.stopped == False:
                threshold = 60  # 閾値
                angular_vel = 0.1  # 角速度
                self.manager.change("find_person")

                # 旋回
                if center < IMAGE_WIDTH / 2 - threshold:
                    twist_msg.angular.z = angular_vel
                elif center > IMAGE_WIDTH / 2 + threshold:
                    twist_msg.angular.z = -angular_vel

                # 直進
                if distance < 50:
                    twist_msg.linear.x = 0.02
                elif distance < 100:
                    twist_msg.linear.x = 1 / distance
                else:
                    self.stopped = True
                    self.manager.change("wall")
                    self.twist_pub.publish(twist_msg)
                    self.rock_sciccors_paper()

        else:
            self.manager.change("lost_person")
            self.stopped = False

        self.twist_pub.publish(twist_msg)
        self.manager.run()

    def rock_sciccors_paper(self):
        msg = Talk()

        msg.text = "じゃんけんしよう"
        self.talk_pub.publish(msg)
        time.sleep(3)

        finger = random.randint(0, 2)  # 0:グー 1:チョキ 2:パー
        if finger == 0:
            finger_text = "グー"
        if finger == 1:
            finger_text = "チョキ"
        if finger == 2:
            finger_text = "パー"
        msg.text = "さいしょはグー、じゃん、けん、" + finger_text
        self.talk_pub.publish(msg)
        time.sleep(3)

        with self.lock:
            gesture = self.gesture

            if not gesture:
                return

            gesture_name = gesture.name

            if (
                (finger == 0 and gesture_name == "Open_Palm")
                or (finger == 1 and gesture_name == "Closed_Fist")
                or (finger == 2 and gesture_name == "Victory")
            ):
                msg.text = "まけたー"
                self.talk_pub.publish(msg)
            elif (
                (finger == 0 and gesture_name == "Victory")
                or (finger == 1 and gesture_name == "Open_Palm")
                or (finger == 2 and gesture_name == "Closed_Fist")
            ):
                msg.text = "かったー"
                self.talk_pub.publish(msg)
            else:
                msg.text = "あいこだね"
                self.talk_pub.publish(msg)


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
