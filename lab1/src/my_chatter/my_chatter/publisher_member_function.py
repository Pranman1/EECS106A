import rclpy
from rclpy.node import Node
from my_chatter_msgs.msg import TimestampString

class MyPublisher(Node):

    def __init__(self):
        super().__init__('my_publisher')
        
        self.publisher_ = self.create_publisher(TimestampString, 'my_chatter_talk', 10)
        
        # We create a timer with a callback (a function that runs automatically when something happens so you don't have to constantly check if something has happened) 
        timer_period = 0 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    # Here we create a message with the counter value appended and publish it
    def timer_callback(self):
        timestamp = TimestampString()
        timestamp.userin = input("Please enter a line of text and press <Enter>:")
        now = self.get_clock().now()
        nanos = now.nanoseconds
        nanos = float(nanos)/1e9
        timestamp.timestamp = nanos
        self.publisher_.publish(timestamp)
        self.get_logger().info('Publishing: "%f"' % timestamp.timestamp)


def main(args=None):
    rclpy.init(args=args)
    my_publisher = MyPublisher()
    # Spin the node so its callbacks are called
    rclpy.spin(my_publisher)


if __name__ == '__main__':
    main()
