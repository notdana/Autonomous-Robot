import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Empty, Int8, Int32
from rcl_interfaces.msg import Log
import os
from dotenv import load_dotenv
import paho.mqtt.client as mqtt
import json
import time

#getting information from .env file
load_dotenv('/home/tubo/tebo1_brain_ws/src/.env')
mqttBroker = os.getenv("mqttBroker")
mqttName = os.getenv("mqttName")
mqttPasswd = os.getenv("mqttPasswd")
robotID = os.getenv("robotID")
root_topic = "Devlacus/Tebo/"+robotID
topics = [root_topic+"/move/#", root_topic+"/info/#", root_topic+"/action/#", root_topic+"/config/#"]

class mqttNode(Node):

    def __init__(self):
        super().__init__('mqtt_node')

        #connecting to mqtt
        self.client = mqtt.Client(client_id=robotID)
        self.client.username_pw_set(mqttName,mqttPasswd)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect(mqttBroker)

        #move topics
        self.manualMove_topic = root_topic+"/move/manual"
        self.tilt_topic = root_topic+"/move/tilt"
        #action topics
        self.goHome_topic = root_topic+"/action/goHome" 
        self.cancelHome_topic = root_topic+"/action/cancelHome" 
        self.start_topic = root_topic+"/action/start"
        self.startMapping_topic = root_topic+"/action/startMapping"
        self.stopMapping_topic = root_topic+"/action/stopMapping"
        self.deleteMap_topic = root_topic+"/action/deleteMap"
        self.led_topic = root_topic+"/action/ledColor" 
        #ignored info topics 
        self.readyState_topic = root_topic+"/info/readyState"
        self.obstacle_topic = root_topic+"/info/obstacle"  
        self.batteryLevel_topic = root_topic+"/info/batteryLevel"  
        self.chargingState_topic = root_topic+"/info/chargingState"  
        self.connectionState_topic = root_topic+"/info/connectionState"
        self.mapState_topic = root_topic+"/info/mapState"
        self.homeState_topic = root_topic+"/info/homeState"
        self.dockState_topic = root_topic+"/info/dockState"
        self.callState_topic = root_topic+"/info/callState"
        self.warning_topic = root_topic+"/info/warning"
        self.ignored_topics= [ self.warning_topic, self.homeState_topic, self.dockState_topic, self.callState_topic, self.mapState_topic, self.readyState_topic, self.obstacle_topic, self.batteryLevel_topic, self.chargingState_topic, self.connectionState_topic]  

        #publishers
        self.move_pub = self.create_publisher(Twist, 'diff_cont/cmd_vel_unstamped', 10)
        self.handleProcess_pub = self.create_publisher(String, 'handle_process', 10)
        self.goHome_pub = self.create_publisher(Empty, 'go_home', 10)
        self.cancelHome_pub = self.create_publisher(Empty, 'cancel_home', 10)
        self.led_pub = self.create_publisher(String, "led_color",10)
        self.tilt_pub = self.create_publisher(Int32, "tilt_angle",10)
        self.led_pub = self.create_publisher(String, "led_color",10)

        #subscribers
        self.readySub = self.create_subscription(String, "ready_state", self.ready_callback, 10)
        self.batterySub = self.create_subscription(Int32, "battery_level", self.battery_callback, 10)
        self.chargingSub = self.create_subscription(Int8, "charging_state", self.charging_callback, 10)
        self.mapStateSub = self.create_subscription(String, "mapState", self.mapState_callback, 10)
        self.homeStateSub = self.create_subscription(String, "homeState", self.homeState_callback, 10)
        self.dockStateSub = self.create_subscription(String, "dockState", self.dockState_callback, 10)
        self.warningSub = self.create_subscription(String, "warning", self.warning_callback, 10)
        self.rosout_sub = self.create_subscription(Log, '/rosout', self.rosout_callback, 10)
        self.front_obstacle_sub = self.create_subscription(Int8, "front_obstacle", self.front_obstacle_callback, 10)
        self.right_obstacle_sub = self.create_subscription(Int8, "right_obstacle", self.right_obstacle_callback, 10)
        self.left_obstacle_sub = self.create_subscription(Int8, "left_obstacle", self.left_obstacle_callback, 10)
        self.back_obstacle_sub = self.create_subscription(Int8, "back_obstacle", self.back_obstacle_callback, 10)

        self.connection_state_timer = self.create_timer(120.0, self.publish_connection)

        #variables
        self.smoothness = 0.0
        self.out_of_map = False
        self.last_move = "stop"
        self.allow_movement = True
        self.call_ended = None
        self.start_timer = 0
        self.end_timer = 0
        self.front_obstacle = False
        self.left_obstacle = False
        self.back_obstacle = False
        self.back_c = 0
        self.front_c = 0
        self.right_c = 0
        self.left_c = 0

    def rosout_callback(self, ros_msg):
        if "out of map bounds" in ros_msg.msg:
            self.get_logger().info("IM OUT OF MAP")
            self.client.publish(self.warning_topic, "unknown area")

    def publish_connection(self):
        self.client.publish(self.connectionState_topic, "connected")


    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("Connected to broker")
            for topic in topics:
                client.subscribe(topic)
            
            self.client.publish(self.connectionState_topic, "connected")

        else:
            self.get_logger().info("Connection failed")
        
    def on_message(self, client, userdata, msg):
        topic = msg.topic
        message = msg.payload.decode()
        if topic not in self.ignored_topics:
            print_msg = "Received: "+message+" on Topic: "+topic
            self.get_logger().info(print_msg)

        if topic==self.goHome_topic:
            process_msg = Empty()
            self.goHome_pub.publish(process_msg)
            self.allow_movement = False

        elif topic==self.cancelHome_topic:
            process_msg = String()
            process_msg.data = "cancelHome"
            self.handleProcess_pub.publish(process_msg)
            process_msg = Empty()
            self.cancelHome_pub.publish(process_msg)
            self.allow_movement = True


        elif topic==self.manualMove_topic:
            self.handle_movement(message)

        elif topic == self.start_topic:
            process_msg = String()
            process_msg.data = "start"
            self.handleProcess_pub.publish(process_msg)

        elif topic == self.tilt_topic:
            tilt_msg = Int32()
            tilt_msg.data = int(message)
            self.tilt_pub.publish(tilt_msg)

        elif topic == self.startMapping_topic:
            process_msg = String()
            process_msg.data = "startMapping"
            self.handleProcess_pub.publish(process_msg)

        elif topic == self.stopMapping_topic:
            process_msg = String()
            process_msg.data = "stopMapping"
            self.handleProcess_pub.publish(process_msg)

        elif topic == self.deleteMap_topic:
            if message == "":
                process_msg = String()
                process_msg.data = "deleteMap"
                self.handleProcess_pub.publish(process_msg)
            else:
                tilt_msg = Int32()
                tilt_msg.data = int(message)
                self.tilt_pub.publish(tilt_msg)


        elif topic == self.led_topic:
            led_msg = String()
            led_msg.data = str(message)
            self.led_pub.publish(led_msg)

        elif topic == self.tilt_topic:
            tilt_msg = Int32()
            tilt_msg.data = int(message)
            self.tilt_pub.publish(tilt_msg)

        elif topic == self.callState_topic:
            if message=="call ended" and not self.call_ended:
                self.call_ended = True
                self.start_timer = time.time()
            if message=="call Started":
                self.call_ended = False

        #topics to ignore
        elif topic in self.ignored_topics:
            pass

        #OTHER topics
        else:
            log_msg = "TOPIC: "+topic+" ERROR: topic unknown"
            self.get_logger().info(log_msg)

    def homeState_callback(self, homeState_msg):
        self.client.publish(self.homeState_topic, homeState_msg.data)

    def dockState_callback(self, dockState_msg):
        self.client.publish(self.dockState_topic, dockState_msg.data)

    def mapState_callback(self, mapState_msg):
        self.client.publish(self.mapState_topic, mapState_msg.data)

    def warning_callback(self, warning_msg):
        self.client.publish(self.warning_topic, warning_msg.data)

    def front_obstacle_callback(self,front_obstacle_msg):
        if front_obstacle_msg.data:
            self.front_c+=1
        else:
            self.front_c = 0

        if self.front_c >= 3:
            self.front_obstacle = True
        else:
            self.front_obstacle = False

        if self.call_ended:
            self.end_timer = time.time()
            time_passed = self.end_timer - self.start_timer
            if time_passed > 180:
                process_msg = Empty()
                self.goHome_pub.publish(process_msg)
                self.allow_movement = False

    def right_obstacle_callback(self,right_obstacle_msg):
        if right_obstacle_msg.data:
            self.right_c+=1
        else:
            self.right_c = 0

        if self.right_c >= 3:
            self.right_obstacle = True
        else:
            self.right_obstacle = False
    def left_obstacle_callback(self,left_obstacle_msg):
        if left_obstacle_msg.data:
            self.left_c+=1
        else:
            self.left_c = 0

        if self.left_c >= 3:
            self.left_obstacle = True
        else:
            self.left_obstacle = False
    def back_obstacle_callback(self,back_obstacle_msg):
        if back_obstacle_msg.data:
            self.back_c+=1
        else:
            self.back_c = 0

        if self.back_c >= 3:
            self.back_obstacle = True
        else:
            self.back_obstacle = False

    def ready_callback(self, ready_msg):
        self.client.publish(self.readyState_topic, ready_msg.data)
        self.allow_movement = True

    def battery_callback(self, battery_msg):
        self.client.publish(self.batteryLevel_topic, battery_msg.data)

    def charging_callback(self, charging_msg):
        self.client.publish(self.chargingState_topic, charging_msg.data)

    def handle_movement(self,move_msg):
        message = move_msg
        if self.allow_movement:
            try:
                if message == "forward":            
                    if not self.front_obstacle:
                        if self.smoothness==0.25:
                            pass
                        elif self.smoothness<0.25:
                            self.smoothness+=0.03
                        self.move(message,self.smoothness,0.0)
                    else:
                        self.smoothness=0
                        self.move(message,0.0,0.0)
                elif message == "back":
                    if not self.back_obstacle:
                        self.move(message,-0.15,0.0)
                    else:
                        self.move(message,0.0,0.0)
                elif message =="left":
                    if not self.left_obstacle:
                        self.move(message,0.0,0.3)
                    else:
                        self.move(message,0.0,0.0)
                elif message =="right":
                    if not self.right_obstacle:
                        self.move(message,0.0,-0.3)
                    else:
                        self.move(message,0.0,0.0)
                elif message =="stop":
                    if self.last_move == "forward":
                        while self.smoothness>0.05:
                            self.smoothness-=0.005
                            time.sleep(0.01)
                            self.move(message, self.smoothness,0.0)
                    else:
                        self.move(message,0.0,0.0)
                    self.smoothness=0

                self.last_move = move_msg

            except Exception as e:
                log_msg = "Exception: "+str(e)+"MESSAGE "+str(message)
                self.get_logger().info(log_msg)
        else:
            self.client.publish(self.warning_topic, "robot not ready")

    def move(self,message, speed_x,speed_z):
        movement = Twist()
        movement.linear.x = speed_x
        movement.angular.z = speed_z
        self.move_pub.publish(movement)


    def start(self):
        self.client.loop_start()
        
    def stop(self):
        self.client.loop_stop()
        self.client.disconnect()


def main(args=None):
    rclpy.init(args=args)
    mqtt_node = mqttNode()
    mqtt_node.start()
    rclpy.spin(mqtt_node)
    mqtt_node.stop()
    mqtt_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
