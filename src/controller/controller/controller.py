import rclpy
from aquestalkpi_ros_msgs.msg import Talk
from geometry_msgs.msg import Twist
from raspimouse_msgs.msg import LightSensors
from rclpy.node import Node

from mediapipe_ros_msgs.msg import BBoxArray

from .state_manager import StateManager

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
        self.timer = self.create_timer(0.01, self.timer_cb)

        self.person = None
        self.distance = 1
        
        # state manager
        self.manager = StateManager()
        self.manager.add("init", self.init_cb)
        self.manager.add("lost_person", self.lost_person_cb)
        self.manager.add("find_person", self.find_person_cb)
        self.manager.add("wall", self.wall_cb)
        
        self.manager.change("init")
        self.manager.run()
        
    def init_cb(self):
        msg = Talk()
        msg.text = "start"
        self.talk_pub.publish(msg)
        
    def lost_person_cb(self):
        msg = Talk()
        msg.text = "where"
        self.talk_pub.publish(msg)
        
    def find_person_cb(self):
        msg = Talk()
        msg.text = "find"
        self.talk_pub.publish(msg)
        
    def wall_cb(self):
        msg = Talk()
        msg.text = "wall"
        self.talk_pub.publish(msg)
        
        

    def bboxes_sub_cb(self, msg):
        persons = sorted(filter(lambda x: x.name == "person", msg.bboxes), key=lambda x: x.score, reverse=True)
        if persons:
            self.person = persons[0]
        else:
            self.person = None

    def light_sensors_sub_cb(self, msg):
        # distance >= 1, 距離が短くなると増大
        self.distance = clamp((msg.forward_r + msg.forward_l) / 2, low=1)

    def timer_cb(self):  # 10msおきに呼ばれる関数
        twist_msg = Twist()
        

        if self.person:  # personが見つかったら
            center = (self.person.xmin + self.person.xmax) / 2  # 中心
            threshold = 60  # 閾値
            angular_vel = 0.1  # 角速度
            self.manager.change("find_person")

            # 旋回
            if center < IMAGE_WIDTH / 2 - threshold:
                twist_msg.angular.z = angular_vel
            elif center > IMAGE_WIDTH / 2 + threshold:
                twist_msg.angular.z = -angular_vel

            # 直進
            if self.distance < 100:
                twist_msg.linear.x = 0.02
            elif self.distance < 100:
                twist_msg.linear.x = 1 / self.distance
            
            if self.distance >= 80:
                self.manager.change("wall")
                
        else:
            self.manager.change("lost_person") 
            
            """ if self.distance >= 100:
                twist_msg.linear.x = 0.0
            elif 50 <= self.distance and self.distance < 100:
                twist_msg.linear.x = 1 / self.distance
            else:
                twist_msg.linear.x = 0.02 """

        self.twist_pub.publish(twist_msg)
        
        
        
        self.manager.run()
        


def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
