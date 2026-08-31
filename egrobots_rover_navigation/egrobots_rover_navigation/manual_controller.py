import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class ManualController(Node):
    def __init__(self):
        super().__init__('manual_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_speed', 0.5)
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.get_logger().info(
            'Manual control ready. w=forward, s=backward, a=left, d=right, x=stop, q=quit'
        )

    def get_key(self):
        # Reads a single keypress from the terminal without needing Enter
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def run(self):
        while rclpy.ok():
            key = self.get_key()
            cmd = Twist()

            if key == 'w':
                cmd.linear.x = self.linear_speed
            elif key == 's':
                cmd.linear.x = -self.linear_speed
            elif key == 'a':
                cmd.angular.z = self.angular_speed
            elif key == 'd':
                cmd.angular.z = -self.angular_speed
            elif key == 'x':
                pass  # cmd stays zeroed — stop
            elif key == 'q':
                self.get_logger().info('Quitting manual control')
                break
            else:
                continue  # ignore unrecognized keys, don't publish

            self.publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ManualController()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        # Publish a final stop command so the robot doesn't keep drifting
        stop_cmd = Twist()
        node.publisher.publish(stop_cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
