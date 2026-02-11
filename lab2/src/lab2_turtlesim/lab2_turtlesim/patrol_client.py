import sys
import rclpy
from rclpy.node import Node

# Import our custom service
from turtle_patrol_interface.srv import Patrol


class TurtlePatrolClient(Node):

    def __init__(self):
        super().__init__('turtle1_patrol_client')


        args = sys.argv[1:]

        name = args[0]
        x = float(args[1])
        y = float(args[2])
        theta = float(args[3])
        vel = float(args[4])
        omega = float(args[5])




        self._service_name = '/turtles/patrol'

        # Create a client for our Patrol service type
        self._client = self.create_client(Patrol, self._service_name)

        # Wait until the server is up (polling loop; logs once per second)
        self.get_logger().info(f"Waiting for service {self._service_name} ...")
        while not self._client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Service {self._service_name} not available, waiting...")


        
        
        # Build request
        req = Patrol.Request()
        req.turtle_name = name
        req.x = x
        req.y = y
        req.theta = theta
        req.vel = vel
        req.omega = omega

        self.get_logger().info(f"Requesting patrol: vel={vel}, omega={omega} for turtle {name} at x={x}, y={y}, theta={theta}")


        # Send request (async under the hood)
        self._future = self._client.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = TurtlePatrolClient()

    # Block here until the service responds (simple for teaching)
    rclpy.spin_until_future_complete(node, node._future)

    if node._future.done():
        result = node._future.result()
        if result is not None:
            # Print the Twist returned by the server
            cmd = result.cmd
            node.get_logger().info(
                f"Service response Twist: lin.x={cmd.linear.x:.2f}, ang.z={cmd.angular.z:.2f}"
            )
        else:
            node.get_logger().error("Service call failed: no result returned.")
    else:
        node.get_logger().error("Service call did not complete.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


