import os
import signal
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Empty
from nav2_simple_commander.robot_navigator import BasicNavigator
import subprocess
import psutil
import serial
import time
import requests
import json
import base64


class ProcessHandlerNode(Node):
    def __init__(self):

        super().__init__('process_handler_node')
        self.navigator = BasicNavigator()

        #subscribers
        self.subscription = self.create_subscription( String, 'handle_process', self.process_callback, 10)
        self.servo_pub = self.create_publisher(Int32, 'tilt_angle', 10)
        self.debug = self.create_publisher(String, 'debugger', 10)

        #publishers
        self.mapStateSub = self.create_publisher(String, "mapState", 10)
        self.handler_state = self.create_publisher(String, "handler_state", 10)
        self.handler_timer = self.create_timer(10.0, self.publish_state)

        self.get_logger().info("handler is handling")

        stop_lidar_motor = 'ros2 service call /stop_motor std_srvs/srv/Empty'
        subprocess.call(stop_lidar_motor, shell=True)

        self.state = "sleeping"

        self.send_map()

    def publish_state(self):
        handler_msg = String()
        handler_msg.data = self.state
        self.handler_state.publish(handler_msg)
    
    def send_debug(self, send_msg):
        debug_msg = String()
        debug_msg.data = send_msg
        self.debug.publish(debug_msg)

    def process_callback(self, msg):

        log_msg = "Recieved msg: "+str(msg.data)
        self.get_logger().info(log_msg)

        if(msg.data == "dock"):

            log_msg = "MSG: "+msg.data+" STATE: "+self.state
            self.get_logger().info(log_msg)

            if self.state == "mapping" or self.state == "saving map" or self.state == "moving":

                if self.map_exists():
                    self.docking()
                    self.state == "docking"
                else:
                    self.save_map()

        elif(msg.data == "sleep"):

            log_msg = "MSG: "+msg.data+" STATE: "+self.state
            self.get_logger().info(log_msg)
            
            if self.state != "sleeping":
                self.sleep()
                self.state = "sleeping"

        elif(msg.data == "start"):
            self.send_debug("MSG: "+msg.data+" STATE: "+self.state)

            if self.state == "sleeping":

                if self.map_exists():
                    map_msg = String()
                    map_msg.data = "map exists"
                    self.mapStateSub.publish(map_msg)
                    self.start()
                    self.state = "moving"

                    #wake up the neck
                    tilt_msg = Int32()
                    tilt_msg.data = 50
                    self.servo_pub.publish(tilt_msg)
                else:
                    map_msg = String()
                    map_msg.data = "no map"
                    self.mapStateSub.publish(map_msg)

        elif(msg.data == "startMapping"):

            log_msg = "MSG: "+msg.data+" STATE: "+self.state
            self.get_logger().info(log_msg)
            
            if self.map_exists():
                map_msg = String()
                map_msg.data = "map exists"
                self.mapStateSub.publish(map_msg)

            elif self.state == "sleeping":
                map_msg = String()
                map_msg.data = "mapping"
                self.mapStateSub.publish(map_msg)
                self.start_map()
                self.state = "mapping"

                #wake up the neck
                tilt_msg = Int32()
                tilt_msg.data = 55
                self.servo_pub.publish(tilt_msg)

            elif self.state == "mapping stopped":
                self.sleep()

        elif(msg.data == "stopMapping"):

            log_msg = "MSG: "+msg.data+" STATE: "+self.state
            self.get_logger().info(log_msg)

            if self.state == "mapping":
                map_msg = String()
                map_msg.data = "mapping stopped"
                self.mapStateSub.publish(map_msg)
                self.stop_map()

        elif(msg.data == "deleteMap"):

            map_msg = String()
            map_msg.data = "map deleted"
            self.mapStateSub.publish(map_msg)

            home_folder = os.path.expanduser('~')

            file_path = os.path.join(home_folder, 'saved_map.pgm')
            if os.path.exists(file_path):
                os.remove(file_path)

            file_path = os.path.join(home_folder, 'saved_map.yaml')
            if os.path.exists(file_path):
                os.remove(file_path)

        else:
            self.get_logger().info("message not recognized")

    def map_exists(self):
        map_path = os.path.expanduser("~")
        map_name = "saved_map.pgm"
        file_path = os.path.join(map_path, map_name)

        if os.path.exists(file_path):
            return True                #neck sleep
        else:
            return False

    def sleep(self):

        kill_nodes = ["tebo_aura_x", "robot_state_pub", "ros2_control_no", "static_transfor", "v4l2_camera_nod", "docking_x", "async_slam_tool" , "component_conta", "return_home_x" , "nav2_remapper_x", "explore"]
        
        if self.state != "sleeping":

            self.state = "sleeping"
            stop_lidar_motor = 'ros2 service call /stop_motor std_srvs/srv/Empty'
            subprocess.call(stop_lidar_motor, shell=True)

            for kill_node in kill_nodes:
                self.kill_process(kill_node)
                self.kill_process("component_conta")

        if self.state == "mapping stopped":
            self.state = "sleeping"
            time.sleep(3)
            self.start_map()


    def start_map(self):
        self.kill_process("rplidar_composi")
        subprocess.Popen(['ros2','launch','blaunch_pkg','tebo_map.launch.py'])

    def stop_map(self):

        kill_nodes = ["nav2_remapper_x", "explore"]
        
        if self.state != "mapping stopped":
            self.state = "mapping stopped"
            for kill_node in kill_nodes:
                self.kill_process(kill_node)

    def save_map(self):

        map_msg = String()
        map_msg.data = "saving map"
        self.mapStateSub.publish(map_msg)

        file_name = "saved_map"
        command1 = f'ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{{name: {{"data": \'~/{file_name}\'}}}}"'
        subprocess.call(command1, shell=True)

        time.sleep(3)

        command2 = f'ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{{filename: \'~/{file_name}.pgm\'}}"'
        subprocess.call(command2, shell=True)

        self.send_map()

        self.docking()
        self.state = "docking"

    def send_map(self):

        map_path = os.path.expanduser("~")
        map_name = "saved_map.pgm"

        image_path = os.path.join(map_path, map_name)
        api_url = "https://tebo.domainenroll.com/api/v1/upload-map"
        #api_url = 'https://hubo2.domainenroll.com/api/v1/save-unknown'

        with open(image_path, "rb") as image_file:
            image_data = image_file.read()

        
        base64_image = base64.b64encode(image_data).decode('utf-8')
        payload = {"robot_uuid": "TEBO-BXAYP-H2WH5-IRONO", "map_file": base64_image}
        #payload = {'employee_id': None, 'guest_image': base64_image}
        json_payload = json.dumps(payload)
        headers = {'Content-Type': 'application/json'}
        
        # Send POST request with JSON payload
        response = requests.post(api_url, data=json_payload, headers=headers)
        print(response)


    def start(self):
        if self.state == "sleeping":
            self.kill_process("rplidar_composi")
            subprocess.Popen(['ros2','launch','blaunch_pkg','start.launch.py'])
            self.state = "starting"

    def docking(self):

        kill_nodes = ["component_conta","obstacle_detect", "tilting_x", "return_home_x" , "nav2_remapper_x", "tebo_aura_x"]

        for kill_node in kill_nodes:
            self.kill_process(kill_node)

        subprocess.Popen(['ros2','run','tebo_main_nodes','docking_x'])
        subprocess.Popen(['ros2','launch','blaunch_pkg','camera.launch.py'])

        self.get_logger().info("Docking...")

    def kill_process(self, name):
        s = "Finding PIDS for "+name
        self.get_logger().info(s)
        pids = self.find_pids(name)

        if pids:
            for pid in pids:
                try:
                    os.kill(pid, signal.SIGTERM)
                    os.kill(pid, signal.SIGKILL)
                    log_msg = f"Killed obstacle_detector_node (PID: {pid})"
                    self.get_logger().info(log_msg)
                except OSError:
                    log_msg = f"Killed obstacle_detector_node (PID: {pid})"
                    self.get_logger().info(log_msg)
        else:
            log_msg = "Process "+str(name)+" not found"
            self.get_logger().info(log_msg)

    def find_pids(self, process_name):
        pids = []
        try:
            output = os.popen(f'pgrep {process_name}').read().strip()
            if output:
                pids = list(map(int, output.split()))
        except OSError:
            self.get_logger().warning(f"Failed to find PIDs of {process_name}")
        return pids
    
    def is_process_running(self, process_name):
        for process in psutil.process_iter(['iter', 'name']):
            if process.info['name'] == process_name:
                self.get_logger().info("process is found")
                return True
        self.get_logger().info("process is not found")
        return False
      

def main(args=None):
    rclpy.init(args=args)
    node = ProcessHandlerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
