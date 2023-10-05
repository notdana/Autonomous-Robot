import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
from std_msgs.msg import String, Int8
import time


class LedNode(Node):

    def __init__(self):
        super().__init__("led_node")

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        self.blue_pin = 17
        self.red_pin = 27
        self.green_pin = 22

        self.led_1 = GPIO.setup(self.blue_pin, GPIO.OUT)
        self.led_2 = GPIO.setup(self.red_pin, GPIO.OUT)
        self.led_3 = GPIO.setup(self.green_pin, GPIO.OUT)

        self.led_sub = self.create_subscription(String, 'led_color', self.led_callback, 10)
        self.dock_sub = self.create_subscription(String, 'handle_process', self.dock_callback, 10)
        self.charging_sub = self.create_subscription(Int8, "charging_state", self.charging_callback, 10)

        self.timer = self.create_timer(1.0, self.led_state)

        self.chosen_color = "cyan"
        self.docking_state = False
        self.charging_state = False
        self.led_set = False

    def led_state(self):

        if self.charging_state:
            GPIO.output(self.blue_pin, GPIO.HIGH)
            GPIO.output(self.red_pin, GPIO.HIGH)
            GPIO.output(self.green_pin, GPIO.LOW)
            self.led_set = False

        elif self.docking_state:
            GPIO.output(self.blue_pin, GPIO.HIGH)
            GPIO.output(self.red_pin, GPIO.HIGH)
            GPIO.output(self.green_pin, GPIO.LOW)
            time.sleep(0.5)
            GPIO.output(self.blue_pin, GPIO.HIGH)
            GPIO.output(self.red_pin, GPIO.HIGH)
            GPIO.output(self.green_pin, GPIO.HIGH)
            time.sleep(0.5)
            self.led_set = False

        elif not self.led_set:
            chosen_color = String()
            chosen_color.data = self.chosen_color
            self.led_callback(chosen_color)
            self.led_set = True


    def dock_callback(self, dock_msg):
        if dock_msg.data == "dock":
            self.docking_state = True

    def charging_callback(self, batt_msg):
        if batt_msg.data == 1:
            self.charging_state = True
            self.docking_state = False
        else:
            self.charging_state = False

    def led_callback(self, led_msg):
        self.chosen_color = led_msg.data
        if led_msg.data == "red":
            #self.get_logger().info("red")
            GPIO.output(self.blue_pin, GPIO.HIGH)
            GPIO.output(self.red_pin, GPIO.LOW)
            GPIO.output(self.green_pin, GPIO.HIGH)

        elif led_msg.data == "blue":
            #self.get_logger().info("blue")
            GPIO.output(self.blue_pin, GPIO.LOW)
            GPIO.output(self.red_pin, GPIO.HIGH)
            GPIO.output(self.green_pin, GPIO.HIGH)

        elif led_msg.data == "green":
            #self.get_logger().info("green")
            GPIO.output(self.blue_pin, GPIO.HIGH)
            GPIO.output(self.red_pin, GPIO.HIGH)
            GPIO.output(self.green_pin, GPIO.LOW)

        elif led_msg.data == "cyan":
            #self.get_logger().info("cyan")
            GPIO.output(self.blue_pin, GPIO.LOW)
            GPIO.output(self.red_pin, GPIO.HIGH)
            GPIO.output(self.green_pin, GPIO.LOW)

        elif led_msg.data == "yellow":
            #self.get_logger().info("yellow")
            GPIO.output(self.blue_pin, GPIO.HIGH)
            GPIO.output(self.red_pin, GPIO.LOW)
            GPIO.output(self.green_pin, GPIO.LOW)

        elif led_msg.data == "purple":
            #self.get_logger().info("purple")
            GPIO.output(self.blue_pin, GPIO.LOW)
            GPIO.output(self.red_pin, GPIO.LOW)
            GPIO.output(self.green_pin, GPIO.HIGH)

        elif led_msg.data == "white":
            #self.get_logger().info("white")
            GPIO.output(self.blue_pin, GPIO.LOW)
            GPIO.output(self.red_pin, GPIO.LOW)
            GPIO.output(self.green_pin, GPIO.LOW)


def main():
    rclpy.init()
    node = LedNode()
    rclpy.spin(node)
    node.destroy()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
