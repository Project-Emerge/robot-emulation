import math
import time
import json
import threading
import random
from typing import Tuple, Dict, List
from urllib.parse import urlparse
from core.robot import Robot
from core.robot import Position
import paho.mqtt.client as mqtt

class RobotWorld:
    def __init__(self, num_robots: int, mqtt_url: str, world_size: float = 10.0):
        self.world_size = world_size
        self.robots: List[Robot] = []
        self.running = False
        self.update_thread = None
        self.mqtt_client = None
        self.mqtt_url = mqtt_url
        
        # Initialize robots in a square formation
        self._initialize_robots(num_robots)
        
        # Setup MQTT
        self._setup_mqtt()
    
    def _initialize_robots(self, num_robots: int):
        """Initialize robots in a square formation"""
        if num_robots <= 0:
            raise ValueError("Number of robots must be positive")
        
        # Calculate grid size for square formation
        grid_size = math.ceil(math.sqrt(num_robots))
        spacing = self.world_size / (grid_size + 1)
        
        for i in range(num_robots):
            row = i // grid_size
            col = i % grid_size
            
            # Calculate position in the square
            x = spacing * (col + 1)
            y = spacing * (row + 1)
            
            # Random initial orientation
            orientation = random.uniform(0, 2 * math.pi)
            
            initial_pos = Position(x, y, orientation)
            robot = Robot(i, initial_pos, self.world_size)
            self.robots.append(robot)
    
    def _parse_mqtt_url(self, url: str) -> Tuple[str, int]:
        """Parse MQTT URL to extract host and port"""
        try:
            parsed = urlparse(url if url.startswith(('mqtt://', 'tcp://')) else f'mqtt://{url}')
            host = parsed.hostname or 'localhost'
            port = parsed.port or 1883
            return host, port
        except Exception:
            print(f"Invalid MQTT URL: {url}, using localhost:1883")
            return 'localhost', 1883
    
    def _setup_mqtt(self):
        """Setup MQTT client and callbacks"""
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_client.on_message = self._on_mqtt_message
        
        try:
            # Parse MQTT URL
            host, port = self._parse_mqtt_url(self.mqtt_url)
            print(f"Connecting to MQTT broker at {host}:{port}")
            
            # Connect to MQTT broker
            self.mqtt_client.connect(host, port)
            self.mqtt_client.loop_start()
        except Exception as e:
            print(f"Failed to connect to MQTT broker: {e}")
            print("Continuing without MQTT...")
    
    def _on_mqtt_connect(self, client, userdata, flags, rc):
        """MQTT connection callback"""
        if rc == 0:
            print("Connected to MQTT broker")
            # Subscribe to command topics for all robots
            for robot in self.robots:
                topic = f"robots/{robot.id}/command"
                client.subscribe(topic)
                print(f"Subscribed to {topic}")
        else:
            print(f"Failed to connect to MQTT broker: {rc}")
    
    def _on_mqtt_message(self, client, userdata, msg):
        """MQTT message callback"""
        try:
            # Extract robot ID from topic
            topic_parts = msg.topic.split('/')
            if len(topic_parts) >= 3 and topic_parts[0] == "robot":
                robot_id = int(topic_parts[1])
                
                # Parse command - expecting JSON format
                try:
                    command = json.loads(msg.payload.decode('utf-8'))
                except json.JSONDecodeError:
                    # Fallback to string command for backward compatibility
                    command = msg.payload.decode('utf-8')
                
                # Find robot and process command
                for robot in self.robots:
                    if robot.id == robot_id:
                        robot.process_command(command)
                        print(f"Robot {robot_id} received command: {command}")
                        break
        except Exception as e:
            print(f"Error processing MQTT message: {e}")
    
    def _publish_robot_status(self, robot: Robot):
        """Publish robot status to MQTT"""
        if self.mqtt_client:
            topic = f"robots/{robot.id}/position"
            message = json.dumps(robot.get_status())
            self.mqtt_client.publish(topic, message)
    
    def _update_loop(self):
        """Main update loop for the simulation"""
        last_time = time.time()
        
        while self.running:
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            
            # Update all robots
            for robot in self.robots:
                robot.update_position(dt)
                self._publish_robot_status(robot)
            
            # Sleep for approximately 1 second
            time.sleep(1.0)
    
    def start(self):
        """Start the robot simulation"""
        if self.running:
            return
        
        self.running = True
        self.update_thread = threading.Thread(target=self._update_loop)
        self.update_thread.daemon = True
        self.update_thread.start()
        print(f"Started simulation with {len(self.robots)} robots")
    
    def stop(self):
        """Stop the robot simulation"""
        self.running = False
        if self.update_thread:
            self.update_thread.join()
        if self.mqtt_client:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
        print("Simulation stopped")
    
    def get_world_status(self) -> Dict:
        """Get status of all robots"""
        return {
            "world_size": self.world_size,
            "num_robots": len(self.robots),
            "robots": [robot.get_status() for robot in self.robots]
        }
    
    def print_status(self):
        """Print current status of all robots"""
        print("\n" + "="*50)
        print(f"ROBOT WORLD STATUS - World Size: {self.world_size}m x {self.world_size}m")
        print("="*50)
        
        for robot in self.robots:
            status = robot.get_status()
            print(f"Robot {robot.id}: "
                  f"Pos({status["x"]:.2f}, {status["y"]:.2f}) "
                  f"Angle: {status["orientation"]:.1f}° ")
