import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
import RPi.GPIO as GPIO
import time

class ServoControllerNode(Node):
    def __init__(self):
        super().__init__('tilting_node')

        self.servo_sub = self.create_subscription(Int32, 'tilt_angle', self.servo_target_callback, 10)
        self.debug = self.create_publisher(String, 'debugger', 10)

        # pin setup
        self.servo_pin = 18 
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.servo_pin, GPIO.OUT)
        self.servo_pwm = GPIO.PWM(self.servo_pin, 50)  
        self.servo_pwm.start(0)  

        #initialize angle
        self.current_angle = 20
        self.set_servo_angle(self.current_angle)
        time.sleep(0.5)
        self.servo_pwm.ChangeDutyCycle(0)

    def send_debug(self, send_msg):
        debug_msg = String()
        debug_msg.data = send_msg
        self.debug.publish(debug_msg)


    def servo_target_callback(self, msg):
        target_angle = self.map_value(msg.data)
        self.move_servo_smoothly(target_angle)

    def move_servo_smoothly(self, target_angle):

        target_angle = self.map_value(target_angle)

        self.send_debug("current angle: "+str(self.current_angle)+" going to: "+str(target_angle))

        step = 0.001 if target_angle >= self.current_angle else -0.001

        while abs(self.current_angle - target_angle) >= 1:
            self.set_servo_angle(self.current_angle)
            self.current_angle += step
            time.sleep(0.000001)  

        self.servo_pwm.ChangeDutyCycle(0)
        self.current_angle = int(target_angle)

    def set_servo_angle(self, angle):
        
        angle = max(0, min(90, angle))

        duty_cycle = 2.5 + 10.0 * angle / 90.0
        self.servo_pwm.ChangeDutyCycle(duty_cycle)
        #self.get_logger().info(f'Set servo angle: {angle}')

    def map_value(self,value):

        from_min, from_max = 0,100
        to_min, to_max = 15, 75

        fraction = (value - from_min) / (from_max - from_min )
        mapped_value = to_min + fraction * (to_max - to_min )

        return mapped_value

def main(args=None):
    rclpy.init(args=args)
    node = ServoControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
    GPIO.cleanup() 

if __name__ == '__main__':
    main()
