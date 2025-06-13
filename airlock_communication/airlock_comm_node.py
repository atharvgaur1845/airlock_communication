#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import requests
import json
import time
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import LaserScan

class AirlockCommunicationNode(Node):
    def __init__(self):
        super().__init__('airlock_comm_node')
        
        # ESP32 endpoint
        self.esp32_base_url = "http://esp32-airlock-controller:8080"
        
        #bot states
        self.current_zone = "unknown"
        self.robot_position = None
        self.airlock_sequence_active = False
        self.last_esp32_response = None
        
        # ROS2 pub
        self.airlock_status_pub = self.create_publisher(String, '/airlock/status', 10)
        self.robot_command_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.zone_status_pub = self.create_publisher(String, '/robot/zone', 10)
        
        # ROS2 sub
        self.robot_position_sub = self.create_subscription(
            Pose2D, '/robot/pose', self.position_callback, 10)
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10)
        self.manual_trigger_sub = self.create_subscription(
            String, '/airlock/manual_request', self.manual_request_callback, 10)
        
        # Timers
        self.status_timer = self.create_timer(0.5, self.poll_airlock_status)
        self.zone_detection_timer = self.create_timer(1.0, self.detect_current_zone)
        self.heartbeat_timer = self.create_timer(2.0, self.send_heartbeat)
        
        # ESP32 connection status
        self.esp32_connected = False
        self.last_successful_ping = 0
        
        self.get_logger().info("Node initialized")
        self.get_logger().info(f"ESP32 endpoint: {self.esp32_base_url}")

    def position_callback(self, msg):
        """Update robot position from odometry/localization"""
        self.robot_position = {
            'x': msg.x,
            'y': msg.y,
            'theta': msg.theta,
            'timestamp': time.time()
        }

    def laser_callback(self, msg):
        """Process laser scan for zone detection and obstacle avoidance"""
        # Simple front obstacle detection
        front_range = msg.ranges[len(msg.ranges)//2]
        if front_range < 0.5:  # 50cm threshold
            self.get_logger().warn("Obstacle detected in front!")

    def manual_request_callback(self, msg):
        """Handle manual airlock passage requests"""
        try:
            request_data = json.loads(msg.data)
            direction = request_data.get('direction', 'forward')
            self.request_airlock_passage(direction, manual=True)
        except json.JSONDecodeError:
            self.get_logger().error("Invalid manual request format")

    def detect_current_zone(self):
        """etect which zone the robot is currently in"""
        if not self.robot_position:
            return
            
        x, y = self.robot_position['x'], self.robot_position['y']
        
        # we will change coordinates according to the setup given
        if x < -2.0:
            new_zone = "external_front"
        elif -2.0 <= x <= 0.0:
            new_zone = "middle"
        elif x > 0.0:
            new_zone = "external_back"
        else:
            new_zone = "unknown"
        
        if new_zone != self.current_zone:
            self.current_zone = new_zone
            zone_msg = String()
            zone_msg.data = new_zone
            self.zone_status_pub.publish(zone_msg)
            self.get_logger().info(f"Robot entered zone: {new_zone}")

    def send_heartbeat(self):
        """Send periodic heartbeat to ESP32"""
        payload = {
            "action": "heartbeat",
            "robot_id": "lynx-robot-main",
            "current_zone": self.current_zone,
            "position": self.robot_position,
            "timestamp": time.time()
        }
        
        response = self.send_request_to_esp32("/airlock/heartbeat", payload)
        if response:
            self.esp32_connected = True
            self.last_successful_ping = time.time()
        else:
            self.esp32_connected = False

    def request_airlock_passage(self, direction="forward", manual=False):
        """Request airlock passage from ESP32"""
        payload = {
            "action": "request_passage",
            "direction": direction,
            "robot_id": "lynx-robot-main",
            "current_zone": self.current_zone,
            "manual_override": manual,
            "timestamp": time.time()
        }
        
        self.get_logger().info(f"Requesting airlock passage: {direction}")
        response = self.send_request_to_esp32("/airlock/request", payload)
        
        if response:
            self.handle_airlock_response(response)
            return response
        else:
            self.get_logger().error("Failed to get airlock response - entering fallback mode")
            self.enter_fallback_mode()
            return None

    def handle_airlock_response(self, response):
        """Process ESP32 response and execute robot actions"""
        status = response.get('status', 'unknown')
        next_action = response.get('next_action', 'wait')
        
        self.last_esp32_response = response
        
        if status == "granted":
            self.get_logger().info(f"Airlock access granted. Next action: {next_action}")
            self.airlock_sequence_active = True
            self.execute_robot_action(next_action)
        elif status == "wait":
            self.get_logger().info("Airlock busy - waiting for clearance")
        elif status == "denied":
            self.get_logger().warn("Airlock access denied")
        else:
            self.get_logger().error(f"Unknown airlock response: {status}")

    def execute_robot_action(self, action):
        """Execute robot movement based on ESP32 instructions"""
        cmd = Twist()
        #here aslo coordinates will change
        if action == "proceed_to_middle":
            cmd.linear.x = 0.2  # 20cm/s forward
            self.get_logger().info("Moving to middle zone")
        elif action == "proceed_to_exit":
            cmd.linear.x = 0.2  # 20cm/s forward
            self.get_logger().info("Moving to exit zone")
        elif action == "stop":
            cmd.linear.x = 0.0
            self.get_logger().info("Stopping robot")
        elif action == "reverse":
            cmd.linear.x = -0.2  # 20cm/s backward
            self.get_logger().info("Reversing robot")
        
        self.robot_command_pub.publish(cmd)

    def poll_airlock_status(self):
        """Periodically check airlock status"""
        if not self.esp32_connected:
            return
            
        response = self.send_request_to_esp32("/airlock/status", {})
        
        if response:
            # Publish status to other ROS nodes
            status_msg = String()
            status_msg.data = json.dumps(response)
            self.airlock_status_pub.publish(status_msg)
            
            # Handle ongoing sequence updates
            if self.airlock_sequence_active:
                self.handle_sequence_update(response)

    def handle_sequence_update(self, status):
        """Handle updates during active airlock sequence"""
        current_state = status.get('current_state', 'unknown')
        
        if current_state == "IDLE_BOTH_CLOSED" and self.airlock_sequence_active:
            self.get_logger().info("Airlock sequence completed")
            self.airlock_sequence_active = False
            # Stop robot
            stop_cmd = Twist()
            self.robot_command_pub.publish(stop_cmd)

    def send_request_to_esp32(self, endpoint, payload, timeout=2.0):
        """Send HTTP request to ESP32 via Husarnet"""
        try:
            url = f"{self.esp32_base_url}{endpoint}"
            
            if payload:
                response = requests.post(url, json=payload, timeout=timeout)
            else:
                response = requests.get(url, timeout=timeout)
            
            if response.status_code == 200:
                return response.json()
            else:
                self.get_logger().error(f"ESP32 returned status {response.status_code}")
                return None
                
        except requests.exceptions.Timeout:
            self.get_logger().error("ESP32 request timeout")
            return None
        except requests.exceptions.ConnectionError:
            self.get_logger().error("ESP32 connection error")
            return None
        except Exception as e:
            self.get_logger().error(f"ESP32 request failed: {e}")
            return None

    def enter_fallback_mode(self):
        """Enter autonomous fallback mode when ESP32 communication fails"""
        self.get_logger().warn("Entering fallback mode - autonomous operation")
        
        # Publish fallback status
        fallback_msg = String()
        fallback_msg.data = json.dumps({
            "status": "fallback_mode",
            "reason": "esp32_communication_failure",
            "timestamp": time.time()
        })
        self.airlock_status_pub.publish(fallback_msg)
        
        # Request both gates open
        self.request_manual_override()

    def request_manual_override(self):
        """Request manual override to open both gates"""
        payload = {
            "action": "manual_override",
            "robot_id": "lynx-robot-main",
            "timestamp": time.time()
        }
        
        # Try to send override request
        self.send_request_to_esp32("/airlock/override", payload)

def main(args=None):
    rclpy.init(args=args)
    
    node = AirlockCommunicationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
