import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class RemapperNode(Node):

    def __init__(self):
        super().__init__('remappper_node')
  
        #topic nav2 publishes to
        self.navControl_sub = self.create_subscription(Twist, 'cmd_vel', self.navControl_callback, 10)
        
        #topic controllers are subscribed to
        self.robot_control_pub = self.create_publisher(Twist, 'diff_cont/cmd_vel_unstamped', 10)

    def navControl_callback(self, msg):
        msg.linear.x = msg.linear.x*1.2
        msg.angular.z = msg.angular.z*1.2
        self.robot_control_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    remapper = RemapperNode()
    rclpy.spin(remapper)
    remapper.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
