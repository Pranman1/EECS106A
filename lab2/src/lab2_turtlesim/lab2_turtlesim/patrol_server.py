import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from turtlesim.srv import TeleportAbsolute
from turtle_patrol_interface.srv import Patrol


class TurtlesPatrolServer(Node):
    def __init__(self):
        super().__init__('turtles_patrol_server')

        self._turtles = {}

        # Publisher: actually drives turtle1
        # self._cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self._srv = self.create_service(Patrol, '/turtles/patrol', self.patrol_callback)

        # Current commanded speeds (what timer publishes)
        self._lin = 0.0
        self._ang = 0.0

        # Timer: publish current speeds at 10 Hz
        self._pub_timer = self.create_timer(0.1, self._publish_current_cmd)

        self.get_logger().info('Turtle1PatrolServer ready (continuous publish mode).')

    # -------------------------------------------------------
    # Timer publishes current Twist
    # -------------------------------------------------------
    def _publish_current_cmd(self):
        for i,v in self._turtles.items():
            msg = Twist()
            msg.linear.x = v[1]
            msg.angular.z = v[2]
            v[0].publish(msg)
            

    # -------------------------------------------------------
    # Service callback: update speeds
    # -------------------------------------------------------
    def patrol_callback(self, request: Patrol.Request, response: Patrol.Response):
        self.get_logger().info(
            f"Patrol request: vel={request.vel:.2f}, omega={request.omega:.2f}"
        )
        turtle_name = request.turtle_name
        vel = request.vel
        omega = request.omega
        x = request.x
        y = request.y
        theta = request.theta

        if turtle_name not in self._turtles:
            store =[]
            cmd_pub = self.create_publisher(Twist, f'/{turtle_name}/cmd_vel', 10)

            store.append(cmd_pub)
            store.append(vel)
            store.append(omega)
            self._turtles[turtle_name] = store
        else:
            self._turtles[turtle_name][1] = vel
            self._turtles[turtle_name][2] = omega


        teleport_client = self.create_client(TeleportAbsolute, f'/{turtle_name}/teleport_absolute')
        teleport_req = TeleportAbsolute.Request()
        teleport_req.x = x
        teleport_req.y = y
        teleport_req.theta = theta
        teleport_client.call_async(teleport_req)



        # Update the speeds that the timer publishes
        lin = float(vel)
        ang = float(omega)

        # Prepare response Twist reflecting current command
        cmd = Twist()
        cmd.linear.x = lin
        cmd.angular.z = ang
        response.cmd = cmd

        self.get_logger().info(
            f"Streaming cmd_vel: lin.x={self._lin:.2f}, ang.z={self._ang:.2f} (10 Hz)"
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TurtlesPatrolServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
