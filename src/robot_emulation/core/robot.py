import math
from typing import Dict
from dataclasses import dataclass


@dataclass
class Position:
    x: float
    y: float
    orientation: float  # in radians

@dataclass
class MotorCommand:
    left: float   # -1 to 1
    right: float  # -1 to 1

class Robot:
    def __init__(self, robot_id: int, initial_position: Position, world_size: float):
        self.id = robot_id
        self.position = initial_position
        self.world_size = world_size
        self.wheel_radius = 0.05  # 5cm wheel radius
        self.wheel_base = 0.2     # 20cm distance between wheels
        self.max_speed = 1.0      # max 1 m/s
        self.left_motor_power = 0.0   # -1 to 1
        self.right_motor_power = 0.0  # -1 to 1
        self.running = False
        
    def process_command(self, command: Dict):
        """Process movement commands with left/right motor values from -1 to 1"""
        try:
            if isinstance(command, dict):
                self.left_motor_power = max(-1.0, min(1.0, float(command.get('left', 0))))
                self.right_motor_power = max(-1.0, min(1.0, float(command.get('right', 0))))
            else:
                # Fallback for string commands for backward compatibility
                if command == 'l':
                    self.left_motor_power = 1.0
                    self.right_motor_power = 0.0
                elif command == 'r':
                    self.left_motor_power = 0.0
                    self.right_motor_power = 1.0
                elif command == 's':
                    self.left_motor_power = 0.0
                    self.right_motor_power = 0.0
                else:
                    self.left_motor_power = 1.0
                    self.right_motor_power = 1.0
        except (ValueError, TypeError):
            print(f"Invalid command for robot {self.id}: {command}")
    
    def update_position(self, dt: float):
        """Update robot position based on differential drive kinematics"""
        # Calculate wheel velocities from motor power (-1 to 1)
        v_left = self.left_motor_power * self.max_speed
        v_right = self.right_motor_power * self.max_speed
        
        # Differential drive kinematics
        if abs(v_left - v_right) < 0.001:  # Moving straight
            velocity = (v_left + v_right) / 2
            self.position.x += velocity * math.cos(self.position.orientation) * dt
            self.position.y += velocity * math.sin(self.position.orientation) * dt
        else:  # Turning
            # Calculate instantaneous center of curvature
            if abs(v_right - v_left) > 0.001:  # Avoid division by zero
                R = self.wheel_base * (v_left + v_right) / (2 * (v_right - v_left))
                omega = (v_right - v_left) / self.wheel_base
                
                # Update orientation
                self.position.orientation += omega * dt
                
                # Update position
                velocity = (v_left + v_right) / 2
                self.position.x += velocity * math.cos(self.position.orientation) * dt
                self.position.y += velocity * math.sin(self.position.orientation) * dt
        
        # Keep orientation in [0, 2π]
        self.position.orientation = self.position.orientation % (2 * math.pi)
        
        # Boundary checking - bounce off walls
        if self.position.x < 0:
            self.position.x = 0
            self.position.orientation = math.pi - self.position.orientation
        elif self.position.x > self.world_size:
            self.position.x = self.world_size
            self.position.orientation = math.pi - self.position.orientation
            
        if self.position.y < 0:
            self.position.y = 0
            self.position.orientation = -self.position.orientation
        elif self.position.y > self.world_size:
            self.position.y = self.world_size
            self.position.orientation = -self.position.orientation
    
    def get_status(self) -> Dict:
        """Get robot status for MQTT publishing"""
        return {
            "robot_id": self.id,
            "x": round(self.position.x, 3),
            "y": round(self.position.y, 3),
            "orientation": round(math.degrees(self.position.orientation), 1)
        }