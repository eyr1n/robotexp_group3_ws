import rclpy                                       
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

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

    def listener_callback(self, joy):
    	dec = 0.1
        twist = Twist()
        if abs(joy.axes[0]>0.1)
        
        	twist.linear.x = 0.2 * joy.axes[1]
        	twist.angular.z = 2 * joy.axes[0]
        	joy.axes[0] = joy.axes[0] - dec
        	dec-=0.1

        	
        self.publisher.publish(twist)
        
def main():
    rclpy.init()
    node = RaspimouseJoy()
    rclpy.spin(node)
    rclpy.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main ()
