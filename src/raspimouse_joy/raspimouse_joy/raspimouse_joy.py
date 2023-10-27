import rclpy                                       
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from aquestalkpi_ros_msgs.msg import Talk # /aquestalkpi_ros
from bbox_msgs.msg import BBoxArray, BBox

class RaspimouseJoy(Node):
    def __init__(self):
        super().__init__('raspimouse_joy')
        self.subscription = self.create_subscription(
            BBoxArray,
            '/bbox_data',
            self.listener_callback,
            10
        )
        self.publisher =self.create_publisher(Twist,'/cmd_vel',10) 
        
        self.talker =self.create_publisher(Talk,'/aquestalkpi_ros',10)
        
        

    def listener_callback(self, bboxarr):
        maxscore = 0
        xmin = 320
        xmax = 320
        for bbox in bboxarr.bboxes:
        
            if bbox.name == "person":
                if bbox.score > maxscore:
                    maxscore = bbox.score
                    xmin = bbox.xmin
                    xmax = bbox.xmax
                    
        center = (xmin+xmax)/2
        twist = Twist()
        if center < 300:
            twist.angular.z = 0.3 #* j
        if center > 340:
            twist.angular.z = -0.3 #* j
        	
        self.publisher.publish(twist)
        
def main():
    rclpy.init()
    node = RaspimouseJoy()
    rclpy.spin(node)
    rclpy.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main ()
