import rclpy
from rclpy.node import Node

from my_chatter_msgs.msg import TimestampString

class MySubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            TimestampString,
            'my_chatter_talk',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        now = self.get_clock().now()
        nanos = now.nanoseconds
        nanos = float(nanos)/1e9
        self.get_logger().info(f"Message: {msg.userin}, Sent at: {msg.timestamp}, Received at: {nanos} ")

def main(args=None):
    rclpy.init(args=args)
    my_subscriber = MySubscriber()
    rclpy.spin(my_subscriber)



if __name__ == '__main__':
    main()
