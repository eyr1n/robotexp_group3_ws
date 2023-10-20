import rclpy                                       
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from aquestalkpi_ros_msgs.msg import Talk # /aquestalkpi_ros

class RaspimouseJoy(Node):
    def __init__(self):
        super().__init__('raspimouse_joy')
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.listener_callback,
            10
        )
        self.publisher =self.create_publisher(Twist,'/cmd_vel',10) 
        
        self.talker =self.create_publisher(Talk,'/aquestalkpi_ros',10)
        
        

    def listener_callback(self, joy):
        twist = Twist()
        twist.linear.x = 0.2 * joy.axes[1]
        twist.angular.z = 2 * joy.axes[0]
        
        if joy.buttons[3]:
            talk = Talk()
            talk.text = "まってー"
            self.talker.publish(talk)

        	
        self.publisher.publish(twist)
        
def main():
    rclpy.init()
    node = RaspimouseJoy()
    rclpy.spin(node)
    rclpy.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main ()
