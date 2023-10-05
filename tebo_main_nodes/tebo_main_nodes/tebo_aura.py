import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Int8, Int32
from gpiozero import MCP3208
import RPi.GPIO as GPIO

class ObstaclesNode(Node):

    def __init__(self):
        super().__init__('obstacles_node')
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.front_obstacle_pub = self.create_publisher(Int8, 'front_obstacle', 10)
        self.right_obstacle_pub = self.create_publisher(Int8, 'right_obstacle', 10)
        self.left_obstacle_pub = self.create_publisher(Int8, 'left_obstacle', 10)
        self.back_obstacle_pub = self.create_publisher(Int8, 'back_obstacle', 10)
        self.plot_pub = self.create_publisher(Int32, 'test_plot', 10)

        #gpio setup for charging_status pin
        GPIO.setmode(GPIO.BCM)
        self.ir_left = 24
        self.ir_right = 23
        GPIO.setup(self.ir_left, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.ir_right, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        self.front_obstacle = False
        self.right_obstacle = False
        self.left_obstacle = False
        self.back_obstacle = False
        self.ur_obstacle_count = 0 



    def scan_callback(self, scan_msg):

        #FRONT OBSTACLE

        #lidar
        front_range = scan_msg.ranges[130:244]
        self.front_obstacle = any(number < 0.4 for number in front_range)

        #ultrasonic
        top_distance = MCP3208(channel=1)._read()

        s = Int32()
        s.data = top_distance
        self.plot_pub.publish(s)

        if top_distance < 500:
            self.ur_obstacle_count+=1
        else:
            self.ur_obstacle_count=0

        #ir
        ir_left_value = GPIO.input(self.ir_left)
        ir_right_value = GPIO.input(self.ir_right)
        #self.get_logger().info("left: "+str(ir_left_value)+" right: "+str(ir_right_value))

        front_obstacle_msg = Int8()
        front_obstacle_msg.data = 1 if self.front_obstacle or self.ur_obstacle_count>= 3 or not ir_left_value or not ir_right_value else 0
        self.front_obstacle_pub.publish(front_obstacle_msg)

        self.get_logger().info("left: "+str(ir_left_value)+" right: "+str(ir_right_value)+" ur: "+str(self.ur_obstacle_count)+" lidar: "+str(self.front_obstacle))

        #TURNING RIGHT OBSTACLE
        self.front_turn_obstacle = any(number < 0.3 for number in front_range)
        right_range = scan_msg.ranges[280:320]
        self.right_obstacle = any(number < 0.35 for number in right_range)

        right_obstacle_msg = Int8()
        right_obstacle_msg.data = 1 if self.right_obstacle or self.front_turn_obstacle or not ir_left_value else 0
        self.right_obstacle_pub.publish(right_obstacle_msg)


        #TURNING LEFT OBSTACLE
        left_range = scan_msg.ranges[40:80]
        self.left_obstacle = any(number < 0.35 for number in left_range)

        left_obstacle_msg = Int8()
        left_obstacle_msg.data = 1 if self.left_obstacle or self.front_turn_obstacle or not ir_right_value else 0
        self.left_obstacle_pub.publish(left_obstacle_msg)

        #BACK OBSTACLE
        back_range1 = scan_msg.ranges[0:30]
        back_range2 = scan_msg.ranges[330:360]
        back_obstacle1 = any(number < 0.5 for number in back_range1)
        back_obstacle2 = any(number < 0.5 for number in back_range2)

        if back_obstacle1 or back_obstacle2:
            self.back_obstacle = True
        else:
            self.back_obstacle = False

        back_obstacle_msg = Int8()
        back_obstacle_msg.data = 1 if self.back_obstacle else 0
        self.back_obstacle_pub.publish(back_obstacle_msg)



def main(args=None):
    rclpy.init(args=args)
    node = ObstaclesNode()
    rclpy.spin(node)
    node.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()       
