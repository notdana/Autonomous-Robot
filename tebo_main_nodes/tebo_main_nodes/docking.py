import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
from time import sleep
import RPi.GPIO as GPIO
from std_msgs.msg import String, Int32
import time

class DockingNode(Node):

    def __init__(self):
        super().__init__('docking_node')

        self.img_sub = self.create_subscription(Image, '/camera/image_raw', self.img_callback, 10)
        self.robot_control_pub = self.create_publisher(Twist, 'diff_cont/cmd_vel_unstamped', 10)
        self.handle_pub= self.create_publisher( String, 'handle_process',10)
        self.dockState_pub = self.create_publisher(String, "dockState", 10)
        self.servo_pub = self.create_publisher(Int32, 'tilt_angle', 10)
        self.cv_bridge = CvBridge()

        self.vm_found = False
        self.vm_found_once = False
        self.x = 0
        self.size = 0

        GPIO.setmode(GPIO.BCM)
        self.charging_state_pin = 4
        GPIO.setup(self.charging_state_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        
        dock_msg = String()
        dock_msg.data = "docking"
        self.dockState_pub.publish(dock_msg)

        #variables
        self.sleep = False
        self.final = 0
        self.stop = False
        self.fix = False
        self.its_charging = False
        self.done = False
        self.neck_sleep = False


    def img_callback(self, msg):

        charging_state = GPIO.input(self.charging_state_pin)
        if charging_state:
            self.its_charging = True
        else:
            self.start = time.time()

        if self.its_charging:
            self.stop = True

            if not self.neck_sleep:
                #neck sleep
                tilt_msg = Int32()
                tilt_msg.data = 0
                self.servo_pub.publish(tilt_msg)
                self.neck_sleep = True

            if self.final <10:
                self.back(0.2)
                self.final +=1
                self.start = time.time()
                return
            else:
                self.end = time.time()
                time_passed = self.end - self.start
                if time_passed < 10 and charging_state:
                    return
                elif time_passed > 10 and charging_state:
                    if self.done:
                        return
                    self.done = True
                    self.stop_moving()
                    if not self.sleep:
                        self.sleep = True
                        dock_msg = String()
                        dock_msg.data = "docking successful"
                        self.dockState_pub.publish(dock_msg)
                        process_msg = String()
                        process_msg.data = "sleep"  
                        self.handle_pub.publish(process_msg)  
                    return
                else:
                    self.get_logger().info("OPS, do it!")
                    if not self.fix:
                        self.fix = True
                        self.go_straight(0.2)
                        self.go_straight(0.2)
                        self.go_straight(0.2)
                        self.go_straight(0.2)
                        time.sleep(1)
                        self.back(0.2)
                        self.back(0.2)
                        self.back(0.2)
                        self.back(0.2)
                        time.sleep(1)
                        self.fix = False           
                  
                return


        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        parameters = cv2.aruco.DetectorParameters_create()
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)


        if ids is not None and not self.its_charging:
            self.vm_found = True
            self.vm_found_once = True
            for i in range(len(ids)):
                # Get the center and size of the marker
                c = corners[i][0]
                x = int((c[0][0] + c[1][0] + c[2][0] + c[3][0]) / 4)
                y = int((c[0][1] + c[1][1] + c[2][1] + c[3][1]) / 4)
                self.size = np.linalg.norm(c[0] - c[1]) 

                if self.size < 250:
                    if x<280:
                        self.turn_left(0.25)
                    elif x>350:
                        self.turn_right(0.25)
                    else:
                        self.back(0.2)
                else:
                    self.back(0.3)
                # if self.size > 270:
                #     self.back(0.1)

                print("X: ", x, " Y: ", y, " Size: ", self.size, " Charging: ", charging_state)
        
        else:
            self.vm_found = False


        if not self.vm_found and not charging_state and not self.stop:
            self.turn_right(0.1)


        cv2.waitKey(1)

    def go_straight(self, speed):
        movement = Twist()
        movement.linear.x = speed
        movement.angular.z = 0.0
        self.robot_control_pub.publish(movement)

    def back(self, speed):
        movement = Twist()
        movement.linear.x = -1*speed
        movement.angular.z = 0.0
        self.robot_control_pub.publish(movement)


    def turn_right(self, speed):
        movement = Twist()
        movement.linear.x = 0.003
        movement.angular.z = -1*speed
        self.robot_control_pub.publish(movement)

    def turn_left(self, speed):
        movement = Twist()
        movement.linear.x = 0.003
        movement.angular.z = speed
        self.robot_control_pub.publish(movement)

    def stop_moving(self):
        movement = Twist()
        movement.linear.x = 0.0
        movement.angular.z = 0.0
        self.robot_control_pub.publish(movement)

def main(args=None):
    rclpy.init(args=args)
    docker = DockingNode()
    rclpy.spin(docker)
    docker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
