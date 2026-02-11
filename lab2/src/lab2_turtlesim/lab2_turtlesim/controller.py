import sys
from click import command
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys

class Controller(Node):

    def __init__(self):

        name = sys.argv[1]
        super().__init__(f'controller_{name}')        
        self.publisher_ = self.create_publisher(Twist, f'{name}/cmd_vel', 10)
        
        # We create a timer with a callback (a function that runs automatically when something happens so you don't have to constantly check if something has happened) 
        timer_period = 0 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    # Here we create a message with the counter value appended and publish it
    def timer_callback(self):
        command = input("Please enter a key stroke and press <enter>:")

        twist  = Twist()
        if command == 'w':
            twist.linear.x = 2.0
        elif command == 's':
            twist.linear.x = -2.0
        elif command == 'a':
            twist.angular.z = 2.0
        elif command == 'd':
            twist.angular.z = -2.0
            
        self.publisher_.publish(twist)
        


def main(args=None):
    rclpy.init(args=args)
    print(args)   
    # name = args[1]
    controller = Controller()
    # Spin the node so its callbacks are called
    rclpy.spin(controller)


if __name__ == '__main__':
    main(sys.argv)
