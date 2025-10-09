#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64, String
import termios
import tty
import sys
import select
import time

class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')
        
        # Publicadores
        self.delta_pub = self.create_publisher(Point, '/piper_cartesian_delta', 10)
        self.target_pub = self.create_publisher(Point, '/piper_cartesian_target', 10)
        self.gripper_pub = self.create_publisher(Float64, '/gripper_command', 10)
        self.command_pub = self.create_publisher(String, '/piper_command', 10)
        
        # Configurar incrementos (en metros)
        self.linear_increment = 0.005  # 1 mm
        self.angular_increment = 0.017  # ~1 grado en radianes
        
        # Estado actual
        self.current_position = Point()
        self.current_position.x = 0.0
        self.current_position.y = 0.0
        self.current_position.z = 0.0
        
        self.get_logger().info("Keyboard Controller initialized")
        self.get_logger().info("Controls:")
        self.get_logger().info("  Movement (X/Y/Z): w/s a/d q/e")
        self.get_logger().info("  Gripper: o (open) c (close)")
        self.get_logger().info("  Home: h")
        self.get_logger().info("  Quit: ESC")
        self.get_logger().info(f"Increment: {self.linear_increment * 1000:.1f} mm")
        
        # Configurar entrada de teclado
        self.old_attr = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        
        # Timer para leer teclado
        self.timer = self.create_timer(0.1, self.read_keyboard)

    def __del__(self):
        # Restaurar configuración del terminal
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_attr)

    def get_key(self):
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            return sys.stdin.read(1)
        return None

    def read_keyboard(self):
        key = self.get_key()
        if key is None:
            return

        publish_delta = False
        publish_gripper = False
        publish_command = False
        
        delta_msg = Point()
        delta_msg.x = 0.0
        delta_msg.y = 0.0
        delta_msg.z = 0.0
        
        gripper_msg = Float64()
        command_msg = String()

        if key in ['w', 'W']:
            delta_msg.x = self.linear_increment
            publish_delta = True
            #self.get_logger().info(f"Moving +X: {self.linear_increment * 1000:.1f} mm")
            
        elif key in ['s', 'S']:
            delta_msg.x = -self.linear_increment
            publish_delta = True
            #self.get_logger().info(f"Moving -X: {self.linear_increment * 1000:.1f} mm")
            
        elif key in ['a', 'A']:
            delta_msg.y = self.linear_increment
            publish_delta = True
            #self.get_logger().info(f"Moving +Y: {self.linear_increment * 1000:.1f} mm")
            
        elif key in ['d', 'D']:
            delta_msg.y = -self.linear_increment
            publish_delta = True
            #self.get_logger().info(f"Moving -Y: {self.linear_increment * 1000:.1f} mm")
            
        elif key in ['q', 'Q']:
            delta_msg.z = self.linear_increment
            publish_delta = True
            #self.get_logger().info(f"Moving +Z: {self.linear_increment * 1000:.1f} mm")
            
        elif key in ['e', 'E']:
            delta_msg.z = -self.linear_increment
            publish_delta = True
            #self.get_logger().info(f"Moving -Z: {self.linear_increment * 1000:.1f} mm")
            
        elif key in ['o', 'O']:
            gripper_msg.data = 0.035  # Abrir
            publish_gripper = True
            #self.get_logger().info("Opening gripper")
            
        elif key in ['c', 'C']:
            gripper_msg.data = 0.0  # Cerrar
            publish_gripper = True
            #self.get_logger().info("Closing gripper")
            
        elif key in ['h', 'H']:
            command_msg.data = "home"
            publish_command = True
            #self.get_logger().info("Going to home position")
            
        elif key == '+':
            self.linear_increment += 0.001
            #self.get_logger().info(f"Increment increased to: {self.linear_increment * 1000:.1f} mm")
            
        elif key == '-':
            self.linear_increment = max(0.001, self.linear_increment - 0.001)
            #self.get_logger().info(f"Increment decreased to: {self.linear_increment * 1000:.1f} mm")
            
        elif key == '\x1b':  # ESC
            #self.get_logger().info("Shutting down...")
            # Restaurar terminal antes de salir
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_attr)
            rclpy.shutdown()
            return

        # Publicar mensajes
        if publish_delta:
            self.delta_pub.publish(delta_msg)
        if publish_gripper:
            self.gripper_pub.publish(gripper_msg)
        if publish_command:
            self.command_pub.publish(command_msg)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Restaurar configuración del terminal
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.old_attr)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()