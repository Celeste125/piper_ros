#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray
import time

class SimpleCartesianController(Node):
    def __init__(self):
        super().__init__('simple_cartesian_controller')
        
        # Publisher para joints
        self.joint_pub = self.create_publisher(Float64MultiArray, '/joint_position_example', 10)
        
        # Subscriber para coordenadas
        self.sub = self.create_subscription(
            Point,
            '/piper_cartesian_target', 
            self.callback,
            10
        )
        
        self.get_logger().info('🚀 SIMPLE Cartesian Controller INICIADO!')
        self.get_logger().info('✅ Listo para recibir coordenadas en /piper_cartesian_target')
        
    def callback(self, msg):
        self.get_logger().info(f'🎯 RECIBIDO: x={msg.x}, y={msg.y}, z={msg.z}')
        
        # MOVIMIENTO SIMPLE - Solo joint1 se mueve
        joint_positions = [0.5, 0.0, 0.0, 0.0, 0.0, 0.0]  # Joint1 a 0.5 radianes
        
        # Crear mensaje
        joint_msg = Float64MultiArray()
        joint_msg.data = joint_positions
        
        # Publicar
        self.joint_pub.publish(joint_msg)
        self.get_logger().info(f'📤 ENVIADO: {joint_positions}')
        
        # Esperar y mover de vuelta
        time.sleep(2.0)
        
        # Mover de vuelta a cero
        joint_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_pub.publish(joint_msg)
        self.get_logger().info('↩️ Regresando a posición cero')

def main():
    rclpy.init()
    node = SimpleCartesianController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()