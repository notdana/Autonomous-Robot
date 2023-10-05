import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Int8, String, Empty
from gpiozero import MCP3208
import RPi.GPIO as GPIO
import datetime
import time

class BatteryNode(Node):

    def __init__(self):
        super().__init__('battery_node')

        #publishers
        self.goHome_pub= self.create_publisher(Empty, 'go_home',10)
        self.handle_pub= self.create_publisher( String, 'handle_process',10)
        self.homeState_pub = self.create_publisher(String, "homeState", 10)
        self.battery_pub = self.create_publisher(Int32, "battery_level", 10)
        self.charging_pub = self.create_publisher(Int8, "charging_state", 10)
        self.warning_pub = self.create_publisher(String, "warning", 10)
        self.timer = self.create_timer(10.0, self.publish_battery)
        self.timer = self.create_timer(10.0, self.publish_charging_state)

        #subscribers
        self.handler_sub = self.create_subscription( String, 'handler_state', self.handler_callback, 10)
        self.ready_sub = self.create_subscription( String, "ready_state", self.ready_callback, 10)

        #gpio setup for charging_status pin
        GPIO.setmode(GPIO.BCM)
        self.charging_pin = 4
        GPIO.setup(self.charging_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        #variables
        self.last_battery_value = None
        self.last_value = 0
        self.charging_state_value = 1
        self.handler_msg = None
        self.ready = False

    def ready_callback(self, ready_msg):
        time.sleep(10)
        self.ready = True

    def handler_callback(self, handler_msg_rec):
        self.handler_msg = handler_msg_rec.data
        if self.handler_msg == "docking" or self.handler_msg == "sleeping":
            self.ready = False

    def publish_charging_state(self):
        charging_state = Int8()
        self.charging_state_value = GPIO.input(self.charging_pin)
        charging_state.data = self.charging_state_value
        self.charging_pub.publish(charging_state)

        # handler_states = ["mapping", "moving"]
        # if self.handler_msg in handler_states and charging_state and self.ready:
        #     process_msg = String()
        #     process_msg.data = "sleep"  
        #     self.handle_pub.publish(process_msg)  

    def publish_battery(self):

        battery_level = Int32()
        battery_voltage = MCP3208(channel=2)._read()
        self.get_logger().info(str(battery_voltage))

        if battery_voltage > 1100:
            battery_value = 100
        elif battery_voltage < 850:
            battery_value = 0
        else:
            battery_value = int(self.map_value(battery_voltage))

        if not self.last_battery_value:
            self.last_battery_value = battery_value

        if self.charging_state_value:
            if battery_value > self.last_battery_value:
                self.last_battery_value = battery_value
        else: 
            if battery_value < self.last_battery_value:
                self.last_battery_value = battery_value
        
        now = datetime.datetime.now()

        battery_level.data = self.last_battery_value
        self.battery_pub.publish(battery_level)

        if self.last_battery_value <= 10:
            warning_msg = String()
            warning_msg.data = "low battery"
            self.warning_pub.publish(warning_msg)

        if self.last_battery_value < 5:
            process_msg = Empty()
            self.goHome_pub.publish(process_msg)

            home_msg = String()
            home_msg.data = "low battery returning homw"
            self.homeState_pub.publish(home_msg)

    def map_value(self,value):

        from_min, from_max = 850,1200
        to_min, to_max = 1,100

        fraction = (value - from_min) / (from_max - from_min )

        mapped_value = to_min + fraction * (to_max - to_min )

        return mapped_value


def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode()
    rclpy.spin(node)
    node.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()       
