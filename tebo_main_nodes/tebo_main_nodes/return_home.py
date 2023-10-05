import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int8, Empty, Int32
from rcl_interfaces.msg import Log
from geometry_msgs.msg import PoseStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from nav_msgs.msg import Odometry
import time

class Nav2Api(Node):
    def __init__(self):
        super().__init__('nav2_api')

        #navigator
        self.navigator = BasicNavigator()

        #publishers
        self.process_pub= self.create_publisher( String, 'handle_process',10)
        self.move_pub = self.create_publisher(Twist, 'diff_cont/cmd_vel_unstamped', 10)
        self.debug = self.create_publisher(String, 'debugger', 10)
        self.ready_pub = self.create_publisher(String, "ready_state", 10)
        self.homeState_pub = self.create_publisher(String, "homeState", 10)

        #subscribers
        self.goHome_sub = self.create_subscription(Empty, 'go_home', self.goHome_callback, 10)
        self.cancelHome_sub = self.create_subscription(Empty, 'cancel_home', self.cancelHome_callback, 10)
        self.odom_sub = self.create_subscription( Odometry,'/diff_cont/odom',  self.odom_callback,10)
        self.rosout_sub = self.create_subscription(Log, '/rosout', self.rosout_callback, 10)

        time.sleep(2)
        
        #Send initial pose
        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = 'map'
        self.initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.initial_pose.pose.position.x = 0.00
        self.initial_pose.pose.position.y = 0.00
        self.initial_pose.pose.position.z = 0.00
        self.initial_pose.pose.orientation.x = 0.00
        self.initial_pose.pose.orientation.y = 0.00
        self.initial_pose.pose.orientation.z - 0.00
        self.initial_pose.pose.orientation.w = 1.00
        self.navigator.setInitialPose(self.initial_pose)
        

        #wait for nav2 to be active
        #self.navigator.waitUntilNav2Active()

        #move robot 1 meter
        time.sleep(2)
        self.move_pub = self.create_publisher(Twist, 'diff_cont/cmd_vel_unstamped', 10)
        for i in range(5000):
            movement = Twist()   
            movement.linear.x = 0.10
            movement.angular.z = 0.0
            self.move_pub.publish(movement)

        #Send ready msg
        ready_msg = String()
        ready_msg.data = "ready"
        self.ready_pub.publish(ready_msg)


        #variables
        self.i = 0
        self.done = False
        self.y_position = 0
        self.x_position = 0
        self.im_here = False

    def rosout_callback(self, ros_msg):
        if ros_msg.msg == "Goal succeeded" and self.done:
             self.get_logger().info("docking")
             process_msg = String()
             process_msg.data = "dock"
             self.process_pub.publish(process_msg)

             home_msg = String()
             home_msg.data = "reached home"
             self.homeState_pub.publish(home_msg)


    def cancelHome_callback(self, h):
        self.get_logger().info("cancelling task")
        self.navigator.cancelTask()
        self.done = False

        home_msg = String()
        home_msg.data = "returning home cancelled"
        self.homeState_pub.publish(home_msg)

    def send_debug(self, send_msg):
        debug_msg = String()
        debug_msg.data = send_msg
        self.debug.publish(debug_msg)

    def odom_callback(self, msg):
        self.y_position = msg.pose.pose.position.y
        self.x_position = msg.pose.pose.position.x

        if self.done:
            if self.x_position < 0.45 and self.x_position > 0.0 and self.y_position < 0.1 and self.y_position > -0.1:
                self.go_home()
                self.im_here = True
                

    def go_home(self):
        if not self.im_here:
            self.get_logger().info("IM HERE!")
            self.navigator.cancelTask()
            process_msg = String()
            process_msg.data = "dock"
            self.process_pub.publish(process_msg)

            home_msg = String()
            home_msg.data = "reached home"
            self.homeState_pub.publish(home_msg)
            

    def goHome_callback(self, home_msg):
        self.get_logger().info("RECIEVED GO HOME")
        if not self.done:
            self.go_to()
            home_msg = String()
            home_msg.data = "returning home"
            self.homeState_pub.publish(home_msg)
            self.done = True

    def go_to(self):

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = 0.25
        goal_pose.pose.position.y = 0.0
        goal_pose.pose.orientation.w = 1.0

        self.navigator.goToPose(goal_pose)


def main(args=None):
    rclpy.init(args=args)
    node = Nav2Api()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
