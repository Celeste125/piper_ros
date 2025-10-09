#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np

class PiperJointControllerSim(Node):
    def __init__(self):
        super().__init__('piper_joint_controller_sim')
        
        # Estado actual de las articulaciones (inicialmente en cero)
        self.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
        # Publicador del estado de joints (para RViz)
        self.joint_state_pub = self.create_publisher(
            JointState, 
            '/joint_states', 
            10
        )
        
        # Suscriptor a comandos de joints (del controlador cartesiano)
        self.joint_cmd_sub = self.create_subscription(
            Float64MultiArray,
            '/joint_position_example',
            self.joint_command_callback,
            10
        )
        
        # Timer para publicar el estado continuamente
        self.timer = self.create_timer(0.1, self.publish_joint_states)  # 10 Hz
        
        self.get_logger().info('Piper Joint Controller Sim iniciado')
    
    def joint_command_callback(self, msg):
        """Recibe comandos de posición de joints y los aplica"""
        if len(msg.data) == 6:
            self.joint_positions = msg.data
            self.get_logger().info(f'Comando recibido: {[f"{x:.3f}" for x in msg.data]}')
        else:
            self.get_logger().warn(f'Comando inválido, esperaba 6 joints, recibió {len(msg.data)}')
    
    def publish_joint_states(self):
        """Publica el estado actual de las articulaciones"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.joint_positions
        self.joint_state_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = PiperJointControllerSim()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()